#!/usr/bin/env python3
"""
make_case.py - generates a two-part 3D-printable case for the ESP32-2432S028
"Cheap Yellow Display" + MikroE ISO 9141 Click, as STL files.

    python3 tools/make_case.py            # writes hardware/*.stl

ALL DIMENSIONS ARE NOMINAL (datasheet / published values), NOT measured on
your boards - check the fit before printing the whole thing (see
hardware/README.md) and change the numbers in the PARAMETERS block below.

Layout (looking down on the open bottom shell):

    +-------------------------------------------+
    |  o                     o   | screw term. ->|   <- cut-out in the wall
    |     [    CYD board    ]    |  [ Click ]    |
    |  o                     o   |               |
    +---------[USB]-----------------------------+   <- cut-out in the wall

The bottom shell holds both boards on posts; the lid has a window for the
display and drops over the shell with a lip (friction fit, no screws).
Geometry is built only from boxes and tubes, so no CAD kernel is needed: walls
with openings are emitted as several boxes around the opening.
"""
import argparse
import math
import struct
import zipfile

# Badge outlines traced from an image by tools/trace_badge.py, if they exist.
# Without them the clover below is drawn from curves instead.
try:
    from badge_data import BADGE_BACK, BADGE_FRONT
except ImportError:                                 # pragma: no cover
    BADGE_BACK = BADGE_FRONT = None

# ---------------------------------------------------------------------------
# PARAMETERS - all in mm. Measure your boards and correct these.
# ---------------------------------------------------------------------------
# Board outline and hole pattern from the manufacturer drawing:
# https://mischianti.org/wp-content/uploads/2025/04/ESP32-2432S028-Cheap-Yellow-Display-Dimensions.jpg.webp
# 4.0 + 78.0 + 4.0 = 86.0 long, 4.0 + 42.0 + 4.0 = 50.0 wide, corners R1.6.
CYD_W, CYD_D, CYD_T = 86.0, 50.0, 1.6      # board outline and PCB thickness
CYD_HOLE_DIA = 3.0                          # mounting hole diameter (not dimensioned: verify)
CYD_HOLE_INSET_X, CYD_HOLE_INSET_Y = 4.0, 4.0   # hole centre from board edge -> 78 x 42 pattern
CYD_SCREEN_W, CYD_SCREEN_D = 57.6, 43.2    # active display area (2.8", 320x240)
CYD_SCREEN_OFF_X, CYD_SCREEN_OFF_Y = 0.0, 0.0   # screen centre vs board centre
CYD_STACK_H = 4.0                           # glass/bezel height above the PCB

# ISO 9141 Click, from iso_9141_click_v101.dxf (outline layer and pad positions):
# outline 25.40 x 28.57 mm; two 8-pin headers 22.86 mm apart, i.e. 1.27 mm from
# the long edges, pins from 3.81 to 21.59 mm along the board; screw terminals
# near the other end. Mounted PINS UP, so the terminals hang underneath.
CLICK_W, CLICK_D, CLICK_T = 25.40, 28.57, 1.6   # X = across the headers, Y = along them
CLICK_PIN_INSET = 1.27                      # header row from the long edge
CLICK_PIN_Y0, CLICK_PIN_Y1 = 3.81, 21.59    # first and last pin along the board
# Terminal height below the board: estimated from the fit-check print (the
# board sat 3.0 mm up and needed "2-4 mm more"). MEASURE IT: it is the only
# number that sets how high the board sits.
CLICK_TERM_STACK = 7.0                      # terminals stand this far below the board
CLICK_TERM_CLEAR = 1.5                      # air under the terminals
CLICK_TERM_X0, CLICK_TERM_X1 = 6.0, 17.0    # terminal bodies occupy this x band
CLICK_BOARD_Z = CLICK_TERM_STACK + CLICK_TERM_CLEAR   # floor to the underside of the PCB
CLICK_POCKET_DEPTH = 3.0                    # back edge slides this far into the pocket
CLICK_RAIL_T = 2.0                          # pocket / guide wall thickness
CLICK_LIP_T = 1.5                           # lip over the board in the pocket
CLICK_LEDGE_LEN = 5.0                       # support ledges at the terminal end
CLICK_NUB_H = 0.5                           # snap nub on the ledges (0 = rely on the lid)
CLICK_WIRE_W = 22.0                         # opening in the wall for the terminal wires
CLICK_PIN_H = 8.0                           # how far the header pins stick up above the
                                            # Click PCB - MEASURE THIS. Add more if plugs
                                            # stay on the pins with the lid closed.

WALL = 2.4                                  # wall thickness
FLOOR = 2.0                                 # floor thickness
GAP = 1.0                                   # clearance left/right of the boards
GAP_Y = 3.0                                 # clearance front/back: also gives the
                                            # sunken screen panel room beside the lip
POST_H_MIN = 4.0                            # minimum clearance under the CYD board
                                            # (the real POST_H is derived below, so the
                                            # screen ends up just under the lid)
POST_OD, POST_ID = 6.0, 2.4                 # post outer / inner diameter (M3 self-tapping)
CLICK_POST_OD, CLICK_POST_ID = 5.0, 2.2
SPIGOT_H = 1.8                              # alignment pin on top of each post, into the
SPIGOT_CLEAR = 0.3                          # board hole (hole diameter minus this)
BOARD_GAP = 4.0                             # between CYD and Click
CLICK_RIGHT_GAP = 6.0                       # right of the Click: room for rail + lid lip
LID_T = 2.0                                 # lid top plate thickness
RECESS_MARGIN = 2.5                         # sunken screen panel vs the active area
RECESS_WALL = 1.6                           # wall between the lid top and the sunken panel
LID_LIP_H = 6.0                             # lip that drops into the shell
LID_CLEAR = 0.3                             # print clearance lip vs inner wall
SCREEN_MARGIN = 1.2                         # window larger than the active area
USB_OPENING = False                         # True = cut a hole for the CYD's USB ports
USB_W, USB_H = 13.0, 9.0                    # that opening: width, and how far it reaches
                                            # below the board (connectors are underneath)
SEG = 48                                    # segments per tube
CLOVER = True                               # emboss an Alfa Romeo clover on the lid
CLOVER_R = 12.5                             # clover size (leaf tips from the centre)
BADGE_INLAY = True                          # True  = badge sunk INTO the lid's top
                                            #         layers, so the surface stays flat
                                            #         (colour change, needs an AMS)
                                            # False = badge raised above the lid
BADGE_INLAY_DEPTH = 0.6                     # how deep the colours reach (3 layers at 0.2)
CLOVER_H = 1.4                              # clover height above the lid surface
CLOVER_TRIANGLE = True                      # solid triangle behind it, like the badge
CLOVER_TRI_H = 0.6                          # triangle height above the lid surface
CLOVER_LEAF_ROUND = 0.35                    # 0 = sharp heart leaves, 1 = round blobs
CLOVER_LEAF_WIDE = 1.12                     # leaves slightly wider than long


def _area(p):
    return sum(p[i][0] * p[(i + 1) % len(p)][1] - p[(i + 1) % len(p)][0] * p[i][1]
               for i in range(len(p))) / 2.0


def _inside(a, b, c, q):
    d1 = (q[0] - b[0]) * (a[1] - b[1]) - (a[0] - b[0]) * (q[1] - b[1])
    d2 = (q[0] - c[0]) * (b[1] - c[1]) - (b[0] - c[0]) * (q[1] - c[1])
    d3 = (q[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (q[1] - a[1])
    return not ((d1 < 0 or d2 < 0 or d3 < 0) and (d1 > 0 or d2 > 0 or d3 > 0))


def triangulate(poly):
    """Ear clipping for a simple polygon; returns a list of triangles.
    Works on indices (not point values), so repeated coordinates are safe."""
    pts = list(poly)
    if _area(pts) < 0:
        pts.reverse()                                 # work counter-clockwise
    idx = list(range(len(pts)))
    out, guard = [], 0
    while len(idx) > 3 and guard < 10000:
        guard += 1
        clipped = False
        for k in range(len(idx)):
            ia, ib, ic = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = pts[ia], pts[ib], pts[ic]
            cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            if cross <= 1e-9:                         # reflex or degenerate corner
                continue
            if any(_inside(a, b, c, pts[j]) for j in idx if j not in (ia, ib, ic)):
                continue
            out.append((a, b, c))
            del idx[k]
            clipped = True
            break
        if not clipped:                               # nothing clippable: give up
            break
    if len(idx) == 3:
        out.append(tuple(pts[i] for i in idx))
    return out


# ---------------------------------------------------------------------------
# Tiny mesh builder: everything is boxes and tubes, emitted as triangles.
# ---------------------------------------------------------------------------
class Mesh:
    def __init__(self):
        self.tris = []

    def tri(self, a, b, c):
        self.tris.append((a, b, c))

    def quad(self, a, b, c, d):
        self.tri(a, b, c)
        self.tri(a, c, d)

    def box(self, x0, y0, z0, x1, y1, z1):
        if x1 <= x0 or y1 <= y0 or z1 <= z0:
            return                                    # skip empty boxes
        p = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
             (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
        self.quad(p[0], p[3], p[2], p[1])             # bottom
        self.quad(p[4], p[5], p[6], p[7])             # top
        self.quad(p[0], p[1], p[5], p[4])             # front
        self.quad(p[1], p[2], p[6], p[5])             # right
        self.quad(p[2], p[3], p[7], p[6])             # back
        self.quad(p[3], p[0], p[4], p[7])             # left

    def wedge(self, x0, y0, x1, y1, z0, z_low, z_high):
        """Block whose top slopes from z_low (at y0) up to z_high (at y1), so a
        board pressed down from above can cam over it."""
        a = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)]
        b = [(x0, y0, z_low), (x1, y0, z_low), (x1, y1, z_high), (x0, y1, z_high)]
        self.quad(a[0], a[3], a[2], a[1])
        self.quad(b[0], b[1], b[2], b[3])
        self.quad(a[0], a[1], b[1], b[0])
        self.quad(a[1], a[2], b[2], b[1])
        self.quad(a[2], a[3], b[3], b[2])
        self.quad(a[3], a[0], b[0], b[3])

    def prism(self, poly, z0, z1):
        """Extrude a simple polygon [(x, y), ...] - concave is fine - between
        two heights. The faces are closed with ear clipping."""
        for a, b, c in triangulate(poly):
            self.tri((a[0], a[1], z1), (b[0], b[1], z1), (c[0], c[1], z1))
            self.tri((a[0], a[1], z0), (c[0], c[1], z0), (b[0], b[1], z0))
        n = len(poly)
        for i in range(n):                            # side walls
            a, b = poly[i], poly[(i + 1) % n]
            self.quad((a[0], a[1], z0), (b[0], b[1], z0), (b[0], b[1], z1), (a[0], a[1], z1))

    def cylinder(self, cx, cy, z0, z1, d, seg=SEG):
        poly = [(cx + d / 2 * math.cos(2 * math.pi * i / seg),
                 cy + d / 2 * math.sin(2 * math.pi * i / seg)) for i in range(seg)]
        self.prism(poly, z0, z1)

    def tube(self, cx, cy, z0, z1, od, idia):
        """Hollow cylinder (a post with a pilot hole for a self-tapping screw)."""
        ro, ri = od / 2.0, idia / 2.0
        for i in range(SEG):
            a0 = 2 * math.pi * i / SEG
            a1 = 2 * math.pi * (i + 1) / SEG
            o0 = (cx + ro * math.cos(a0), cy + ro * math.sin(a0))
            o1 = (cx + ro * math.cos(a1), cy + ro * math.sin(a1))
            i0 = (cx + ri * math.cos(a0), cy + ri * math.sin(a0))
            i1 = (cx + ri * math.cos(a1), cy + ri * math.sin(a1))
            self.quad((o0[0], o0[1], z0), (o1[0], o1[1], z0),
                      (o1[0], o1[1], z1), (o0[0], o0[1], z1))          # outside
            self.quad((i1[0], i1[1], z0), (i0[0], i0[1], z0),
                      (i0[0], i0[1], z1), (i1[0], i1[1], z1))          # inside
            self.quad((o1[0], o1[1], z1), (i1[0], i1[1], z1),
                      (i0[0], i0[1], z1), (o0[0], o0[1], z1))          # top ring
            self.quad((i0[0], i0[1], z0), (i1[0], i1[1], z0),
                      (o1[0], o1[1], z0), (o0[0], o0[1], z0))          # bottom ring

    def wall_x(self, x0, x1, y0, y1, z0, z1, openings=()):
        """Wall along X; each opening = (x_centre, width, z_bottom, z_top)."""
        cuts = sorted(openings, key=lambda o: o[0])
        x = x0
        for cx, w, oz0, oz1 in cuts:
            self.box(x, y0, z0, cx - w / 2, y1, z1)                # solid up to the opening
            self.box(cx - w / 2, y0, z0, cx + w / 2, y1, oz0)      # below it
            self.box(cx - w / 2, y0, oz1, cx + w / 2, y1, z1)      # above it
            x = cx + w / 2
        self.box(x, y0, z0, x1, y1, z1)                            # rest of the wall

    def wall_y(self, y0, y1, x0, x1, z0, z1, openings=()):
        """Wall along Y; each opening = (y_centre, width, z_bottom, z_top)."""
        cuts = sorted(openings, key=lambda o: o[0])
        y = y0
        for cy, w, oz0, oz1 in cuts:
            self.box(x0, y, z0, x1, cy - w / 2, z1)
            self.box(x0, cy - w / 2, z0, x1, cy + w / 2, oz0)
            self.box(x0, cy - w / 2, oz1, x1, cy + w / 2, z1)
            y = cy + w / 2
        self.box(x0, y, z0, x1, y1, z1)

    def plate_with_window(self, x0, y0, x1, y1, z0, z1, wx0, wy0, wx1, wy1):
        """Flat plate with a rectangular hole, as four boxes around the hole."""
        self.plate_with_windows(x0, y0, x1, y1, z0, z1, [(wx0, wy0, wx1, wy1)])

    def plate_with_windows(self, x0, y0, x1, y1, z0, z1, windows):
        """Plate with several holes that don't overlap in X (left to right)."""
        wins = sorted(windows, key=lambda w: w[0])
        x = x0
        for wx0, wy0, wx1, wy1 in wins:
            self.box(x, y0, z0, wx0, y1, z1)          # strip left of the hole
            self.box(wx0, y0, z0, wx1, wy0, z1)       # below it
            self.box(wx0, wy1, z0, wx1, y1, z1)       # above it
            x = wx1
        self.box(x, y0, z0, x1, y1, z1)               # strip right of the last hole

    def bounds(self):
        xs = [v[0] for t in self.tris for v in t]
        ys = [v[1] for t in self.tris for v in t]
        zs = [v[2] for t in self.tris for v in t]
        return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))

    def save(self, path, name):
        with open(path, "wb") as f:
            f.write(struct.pack("<80sI", name.encode()[:80], len(self.tris)))
            for a, b, c in self.tris:
                u = [b[i] - a[i] for i in range(3)]
                v = [c[i] - a[i] for i in range(3)]
                n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                     u[0] * v[1] - u[1] * v[0]]
                ln = math.sqrt(sum(k * k for k in n)) or 1.0
                f.write(struct.pack("<3f", *[k / ln for k in n]))
                for p in (a, b, c):
                    f.write(struct.pack("<3f", *p))
                f.write(struct.pack("<H", 0))


def write_3mf(path, parts, name="CYD case lid"):
    """Write a 3MF holding several meshes as PARTS OF ONE OBJECT (3MF
    components), so a slicer shows one object you can give a filament per part.
    parts = [(part name, mesh, "#RRGGBB"), ...] in shared coordinates."""
    core = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
    res, comps = [], []
    mats = ['<basematerials id="1">']
    for i, (pname, _mesh, colour) in enumerate(parts):
        mats.append('  <base name="%s" displaycolor="%sFF"/>' % (pname, colour))
    mats.append('</basematerials>')
    for i, (pname, mesh, _c) in enumerate(parts):
        oid = i + 2                                   # 1 = the material group
        verts, index = [], {}
        tris = []
        for tri in mesh.tris:
            ids = []
            for v in tri:
                key = (round(v[0], 5), round(v[1], 5), round(v[2], 5))
                if key not in index:
                    index[key] = len(verts)
                    verts.append(key)
                ids.append(index[key])
            if len(set(ids)) == 3:                    # skip degenerate triangles
                tris.append(ids)
        res.append(
            '<object id="%d" type="model" name="%s" pid="1" pindex="%d"><mesh>\n'
            '<vertices>\n%s\n</vertices>\n<triangles>\n%s\n</triangles>\n'
            '</mesh></object>'
            % (oid, pname, i,
               "\n".join('<vertex x="%.4f" y="%.4f" z="%.4f"/>' % v for v in verts),
               "\n".join('<triangle v1="%d" v2="%d" v3="%d"/>' % tuple(t) for t in tris)))
        comps.append('<component objectid="%d"/>' % oid)
    top = len(parts) + 2
    res.append('<object id="%d" type="model" name="%s"><components>%s</components></object>'
               % (top, name, "".join(comps)))
    model = ('<?xml version="1.0" encoding="UTF-8"?>\n'
             '<model unit="millimeter" xml:lang="en-US" xmlns="%s">\n'
             '<resources>\n%s\n%s\n</resources>\n'
             '<build><item objectid="%d"/></build>\n</model>\n'
             % (core, "\n".join(mats), "\n".join(res), top))
    ct = ('<?xml version="1.0" encoding="UTF-8"?>\n'
          '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
          '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
          '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
          '</Types>\n')
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
            'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>'
            '</Relationships>\n')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", ct)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", model)
    return sum(len(m.tris) for _n, m, _c in parts)


# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------
INNER_W = GAP + CYD_W + BOARD_GAP + CLICK_W + CLICK_RIGHT_GAP   # inside of the shell
INNER_D = max(CYD_D, CLICK_D) + 2 * GAP_Y
# The Click's screw terminals are the tallest thing inside, so they set the
# shell height; the display sits lower and the lid is sunk down over it.
# The lid is closed over the Click, so its pins (and anything plugged on them)
# set the inner height. The CYD posts are then made tall enough to bring the
# display up to 2 mm under the lid, instead of leaving the screen in a well.
CLICK_TOP = CLICK_BOARD_Z + CLICK_T + CLICK_PIN_H + 1.0    # floor to above the pins
INNER_H = max(POST_H_MIN + CYD_T + CYD_STACK_H + 2.0, CLICK_TOP)
POST_H = INNER_H - 2.0 - CYD_T - CYD_STACK_H           # -> display 2 mm under the lid
DISPLAY_TOP = POST_H + CYD_T + CYD_STACK_H             # floor to the glass
RECESS_DEPTH = max(0.0, INNER_H - DISPLAY_TOP - 2.0)   # 0 here: the lid can stay flat
OUTER_W = INNER_W + 2 * WALL
OUTER_D = INNER_D + 2 * WALL
SHELL_H = FLOOR + INNER_H                              # shell wall height

CYD_X0 = WALL + GAP                                    # board origins inside
CYD_Y0 = WALL + (INNER_D - CYD_D) / 2 + 0.0
CLICK_X0 = CYD_X0 + CYD_W + BOARD_GAP
CLICK_Y0 = WALL + (INNER_D - CLICK_D) / 2


def build_bottom():
    m = Mesh()
    m.box(0, 0, 0, OUTER_W, OUTER_D, FLOOR)                       # floor
    z0, z1 = FLOOR, SHELL_H
    # Only one opening: the wires off the Click's screw terminals. No USB
    # cut-out (set USB_OPENING = True to get one back), and the other three
    # walls are solid.
    click_x = CLICK_X0 + CLICK_W / 2
    board_z = FLOOR + POST_H                                      # underside of the CYD
    cuts = [(click_x, CLICK_WIRE_W, FLOOR + 0.5, FLOOR + CLICK_BOARD_Z)]
    if USB_OPENING:
        cuts.append((CYD_X0 + CYD_W / 2, USB_W, board_z - USB_H, board_z + 1.0))
    m.wall_x(0, OUTER_W, 0, WALL, z0, z1, openings=cuts)
    # Back wall is solid: the K-line and 12 V wires go to the Click's screw
    # terminals, so they already leave through the Click opening in the front
    # wall. A slot here would also sit right behind the CYD's micro-SD slot.
    m.wall_x(0, OUTER_W, OUTER_D - WALL, OUTER_D, z0, z1)
    m.wall_y(0, OUTER_D, 0, WALL, z0, z1)                         # left wall
    m.wall_y(0, OUTER_D, OUTER_W - WALL, OUTER_W, z0, z1)         # right wall
    spigot = CYD_HOLE_DIA - SPIGOT_CLEAR                          # pin into the board hole
    for dx in (CYD_HOLE_INSET_X, CYD_W - CYD_HOLE_INSET_X):       # CYD posts
        for dy in (CYD_HOLE_INSET_Y, CYD_D - CYD_HOLE_INSET_Y):
            m.tube(CYD_X0 + dx, CYD_Y0 + dy, FLOOR, FLOOR + POST_H, POST_OD, POST_ID)
            m.tube(CYD_X0 + dx, CYD_Y0 + dy, FLOOR + POST_H, FLOOR + POST_H + SPIGOT_H,
                   spigot, POST_ID)
    click_holder(m)
    return m


def click_holder(m, board_z=None, ox=None, oy=None):
    """Slide-in holder for the Click, mounted PINS UP.

    The two 8-pin headers sit only 1.27 mm from the long edges, so nothing may
    grip those edges along the board: a long rail would ride over the pins
    while sliding in. Instead the BACK edge slides 3 mm into a pocket (that
    end of the board is free of pins), and the front end is then pressed down
    onto two ledges whose nubs snap over it. The terminals hang below the
    board, between the ledges.
    """
    x0 = CLICK_X0 if ox is None else ox
    y0 = CLICK_Y0 if oy is None else oy
    x1, y1 = x0 + CLICK_W, y0 + CLICK_D
    base = FLOOR if board_z is None else 2.0
    bz = base + (CLICK_BOARD_Z if board_z is None else board_z)   # underside of the board
    top = bz + CLICK_T + 0.3                        # top of the board + slide clearance
    py0 = y1 - CLICK_POCKET_DEPTH                   # mouth of the pocket

    # pocket: support below the board, lip above it, across the full width
    m.box(x0, py0, bz - 1.2, x1, y1, bz)
    m.box(x0, py0, top, x1, y1, top + CLICK_LIP_T)
    # back wall and two side guides (outside the board edges, clear of the pins)
    m.box(x0 - CLICK_RAIL_T, y1, base, x1 + CLICK_RAIL_T, y1 + CLICK_RAIL_T, top + CLICK_LIP_T)
    m.box(x0 - CLICK_RAIL_T, py0, base, x0, y1, top + CLICK_LIP_T)
    m.box(x1, py0, base, x1 + CLICK_RAIL_T, y1, top + CLICK_LIP_T)
    # columns carrying the pocket, so it is not floating
    m.box(x0, py0, base, x0 + 2.0, y1, bz - 1.2)
    m.box(x1 - 2.0, py0, base, x1, y1, bz - 1.2)

    # front ledges, clear of the terminal bodies; nubs clear of the pin columns
    pin_l, pin_r = CLICK_PIN_INSET + 1.2, CLICK_W - CLICK_PIN_INSET - 1.2
    for lx0, lx1, nx0, nx1 in (
            (x0, x0 + CLICK_TERM_X0 - 1.0, x0 + pin_l, x0 + CLICK_TERM_X0 - 1.0),
            (x0 + CLICK_TERM_X1 + 1.0, x1, x0 + CLICK_TERM_X1 + 1.0, x0 + pin_r)):
        m.box(lx0, y0, base, lx1, y0 + CLICK_LEDGE_LEN, bz)          # pillar + ledge
        if CLICK_NUB_H > 0:                                          # ramped snap nub
            m.wedge(nx0, y0, nx1, y0 + 2.0, bz + CLICK_T + 0.3,
                    bz + CLICK_T + 0.3, bz + CLICK_T + 0.3 + CLICK_NUB_H)


def leaf_outline(n=120):
    """One clover leaflet: a heart curve blended towards an ellipse, which
    rounds the lobes and shallows the notch - a heart alone reads as a playing
    card, a circle alone as a blob. Tip at the bottom (towards the centre)."""
    k, wide = CLOVER_LEAF_ROUND, CLOVER_LEAF_WIDE
    pts = []
    for i in range(n):
        t = 2 * math.pi * i / n
        hx = 16 * math.sin(t) ** 3 / 17.0
        hy = (13 * math.cos(t) - 5 * math.cos(2 * t)
              - 2 * math.cos(3 * t) - math.cos(4 * t)) / 17.0
        pts.append((hx, hy))
    cy = sum(p[1] for p in pts) / n
    out = []
    for i, (hx, hy) in enumerate(pts):
        t = 2 * math.pi * i / n
        ex, ey = 0.78 * math.sin(t), cy + 0.80 * math.cos(t)
        out.append((((1 - k) * hx + k * ex) * wide, (1 - k) * hy + k * ey))
    return out


def clover(m, cx, cy, z0, z1, r):
    """Alfa Romeo quadrifoglio: four leaflets around a centre with a stem, on a
    solid triangle. Drawn from curves, so it suggests the badge, not copies it."""
    if CLOVER_TRIANGLE:                                  # solid triangle, lower relief
        R = r * 2.05
        m.prism([(cx + R * math.cos(math.radians(a)), cy + R * math.sin(math.radians(a)))
                 for a in (90, 210, 330)], LID_T, LID_T + CLOVER_TRI_H)
    leaf = leaf_outline()
    oy = cy + r * 0.10                                   # a little high: stem sits below
    scale, dist = r * 0.50, r * 0.58
    for ang in (45, 135, 225, 315):
        a = math.radians(ang)
        ca, sa = math.cos(a - math.pi / 2), math.sin(a - math.pi / 2)
        dx, dy = dist * math.cos(a), dist * math.sin(a)
        m.prism([(cx + dx + scale * (x * ca - y * sa),
                  oy + dy + scale * (x * sa + y * ca)) for x, y in leaf], z0, z1)
    stem = []                                            # tapered, slightly curved
    for sgn in (-1, 1):
        rng = range(13) if sgn < 0 else range(12, -1, -1)
        for i in rng:
            t = i / 12.0
            bx = (2 * (1 - t) * t * 0.08 + t * t * 0.12) * r
            by = (2 * (1 - t) * t * (-0.50) + t * t * (-1.02)) * r
            w = (0.14 - 0.08 * t) * r
            stem.append((cx + bx + sgn * w / 2, oy + by))
    m.prism(stem, z0, z1)


def lid_lip(m):
    """Lip around the lid that drops into the shell and locates it."""
    lip_x0 = WALL + LID_CLEAR
    lip_y0 = WALL + LID_CLEAR
    lip_x1 = OUTER_W - WALL - LID_CLEAR
    lip_y1 = OUTER_D - WALL - LID_CLEAR
    t = WALL * 0.6
    z0, z1 = -LID_LIP_H, 0.0
    m.box(lip_x0, lip_y0, z0, lip_x1, lip_y0 + t, z1)
    m.box(lip_x0, lip_y1 - t, z0, lip_x1, lip_y1, z1)
    m.box(lip_x0, lip_y0, z0, lip_x0 + t, lip_y1, z1)
    m.box(lip_x1 - t, lip_y0, z0, lip_x1, lip_y1, z1)


def build_lid(with_badge=True):
    """Lid top at z = 0..LID_T; the screen panel is sunk by RECESS_DEPTH so the
    window sits just above the glass while the lid clears the Click terminals."""
    m = Mesh()
    sx = CYD_X0 + CYD_W / 2 + CYD_SCREEN_OFF_X                    # window centre
    sy = CYD_Y0 + CYD_D / 2 + CYD_SCREEN_OFF_Y
    wx0 = sx - (CYD_SCREEN_W + SCREEN_MARGIN) / 2
    wx1 = sx + (CYD_SCREEN_W + SCREEN_MARGIN) / 2
    wy0 = sy - (CYD_SCREEN_D + SCREEN_MARGIN) / 2
    wy1 = sy + (CYD_SCREEN_D + SCREEN_MARGIN) / 2
    rx0, ry0 = wx0 - RECESS_MARGIN, wy0 - RECESS_MARGIN            # sunken panel area
    rx1, ry1 = wx1 + RECESS_MARGIN, wy1 + RECESS_MARGIN
    d = RECESS_DEPTH
    if d < 0.6:                                   # nothing to sink: flat lid
        m.plate_with_window(0, 0, OUTER_W, OUTER_D, 0, LID_T, wx0, wy0, wx1, wy1)
        if with_badge:
            lid_badge(m, wx1)
        lid_lip(m)
        return m
    # top plate with a big hole over the display, then the sunken panel with
    # the window, then four walls joining the two.
    m.plate_with_window(0, 0, OUTER_W, OUTER_D, 0, LID_T, rx0, ry0, rx1, ry1)
    m.plate_with_window(rx0, ry0, rx1, ry1, -d, -d + LID_T, wx0, wy0, wx1, wy1)
    m.box(rx0, ry0, -d, rx0 + RECESS_WALL, ry1, LID_T)
    m.box(rx1 - RECESS_WALL, ry0, -d, rx1, ry1, LID_T)
    m.box(rx0, ry0, -d, rx1, ry0 + RECESS_WALL, LID_T)
    m.box(rx0, ry1 - RECESS_WALL, -d, rx1, ry1, LID_T)
    if with_badge:
        lid_badge(m, rx1)
    lid_lip(m)
    return m


def build_lid_embossed():
    """Lid with the badge standing proud, for printing in a single colour."""
    global BADGE_INLAY
    was, BADGE_INLAY = BADGE_INLAY, False
    try:
        return build_lid(with_badge=True)
    finally:
        BADGE_INLAY = was


def badge_layer(layer):
    """One layer of the badge as its own mesh, in the same coordinates as the
    lid: load it in the slicer as a part of the lid and give it its own
    filament. layer = "triangle" or "clover".

    With BADGE_INLAY the parts sit INSIDE the lid's top layers and all end
    flush at the lid surface, so the lid stays flat and only the colour
    changes. Parts overlap the lid (and the clover overlaps the triangle);
    slicers give an overlap to the part listed last, which is why the clover
    is written after the triangle."""
    m = Mesh()
    sx = CYD_X0 + CYD_W / 2 + CYD_SCREEN_OFF_X
    wx1 = sx + (CYD_SCREEN_W + SCREEN_MARGIN) / 2
    if RECESS_DEPTH >= 0.6:
        wx1 += RECESS_MARGIN
    lid_badge(m, wx1, only=layer)
    return m


def lid_badge(m, free_x0, only=None):
    """Badge on the free part of the lid, between the screen area and the right
    wall: the traced outlines when tools/badge_data.py exists, else the clover
    drawn from curves."""
    if not CLOVER:
        return
    margin = 2.5
    x0, x1 = free_x0 + margin, OUTER_W - WALL - margin
    y0, y1 = WALL + margin, OUTER_D - WALL - margin
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    if BADGE_FRONT:
        pts = [p for poly in (BADGE_FRONT + (BADGE_BACK or [])) for p in poly]
        bw = max(p[0] for p in pts) - min(p[0] for p in pts)
        bh = max(p[1] for p in pts) - min(p[1] for p in pts)
        scale = min((x1 - x0) / bw, (y1 - y0) / bh)
        if BADGE_INLAY:                               # flush with the lid surface
            z0, z1 = LID_T - BADGE_INLAY_DEPTH, LID_T
            if only in (None, "triangle"):
                for poly in (BADGE_BACK or []):
                    m.prism([(cx + x * scale, cy + y * scale) for x, y in poly], z0, z1)
            if only in (None, "clover"):
                for poly in BADGE_FRONT:
                    m.prism([(cx + x * scale, cy + y * scale) for x, y in poly], z0, z1)
            return
        if only in (None, "triangle"):                # raised badge
            for poly in (BADGE_BACK or []):
                m.prism([(cx + x * scale, cy + y * scale) for x, y in poly],
                        LID_T, LID_T + CLOVER_TRI_H)
        if only in (None, "clover"):
            z0 = LID_T + CLOVER_TRI_H if only == "clover" else LID_T
            for poly in BADGE_FRONT:
                m.prism([(cx + x * scale, cy + y * scale) for x, y in poly],
                        z0, LID_T + CLOVER_H)
        return
    if only in (None, "clover"):
        clover(m, cx, cy, LID_T, LID_T + CLOVER_H, CLOVER_R)


def build_fitcheck():
    """Small coupon with the same post pattern as the shell: print this first
    (a few minutes) to check the hole spacing before printing the whole case."""
    m = Mesh()
    posts = [(CYD_X0 + dx, CYD_Y0 + dy, POST_OD, POST_ID)
             for dx in (CYD_HOLE_INSET_X, CYD_W - CYD_HOLE_INSET_X)
             for dy in (CYD_HOLE_INSET_Y, CYD_D - CYD_HOLE_INSET_Y)]

    pad = 4.0
    x0 = min(p[0] for p in posts) - POST_OD / 2 - pad
    x1 = max(p[0] for p in posts) + POST_OD / 2 + pad
    y0 = min(p[1] for p in posts) - POST_OD / 2 - pad
    y1 = max(p[1] for p in posts) + POST_OD / 2 + pad
    x1 = max(x1, CLICK_X0 + CLICK_W + CLICK_RAIL_T + 2)
    m.box(x0, y0, 0, x1, y1, 2.0)                      # thin base plate
    ph = 4.0                                           # short posts: this coupon only
    for cx, cy, od, idia in posts:                     # checks the hole PATTERN
        m.tube(cx, cy, 2.0, 2.0 + ph, od, idia)
        m.tube(cx, cy, 2.0 + ph, 2.0 + ph + SPIGOT_H,
               CYD_HOLE_DIA - SPIGOT_CLEAR, idia)      # same alignment pins as the shell
    return m


def build_clickfit():
    """Just the Click holder on a small plate, at a low board height: print this
    to check that the board slides in and the nubs hold it, before the shell."""
    m = Mesh()
    pad = 4.0                       # same board height as the real shell, so the
    bz = CLICK_BOARD_Z              # terminals and the slide-in can be checked
    w = CLICK_W + 2 * CLICK_RAIL_T + 2 * pad
    d = CLICK_D + CLICK_RAIL_T + 2 * pad
    m.box(0, 0, 0, w, d, 2.0)
    click_holder(m, board_z=bz, ox=pad + CLICK_RAIL_T, oy=pad)
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", default="hardware")
    args = ap.parse_args()

    parts = [("case_bottom", build_bottom()),
             ("case_lid", build_lid(with_badge=False)),          # base part for the AMS
             ("case_lid_embossed", build_lid_embossed()),
             ("case_fitcheck", build_fitcheck()), ("case_clickfit", build_clickfit())]
    if CLOVER:
        parts += [("case_lid_badge_triangle", badge_layer("triangle")),
                  ("case_lid_badge_clover", badge_layer("clover"))]
    if CLOVER:
        lid_parts = [("lid", build_lid(with_badge=False), "#C8C8CD"),
                     ("badge triangle", badge_layer("triangle"), "#F2F2F2"),
                     ("badge clover", badge_layer("clover"), "#0F7A32")]
        n = write_3mf("%s/case_lid.3mf" % args.out_dir, lid_parts)
        print("%-26s %5d triangles  3 parts in one object (lid + triangle + clover)"
              % ("case_lid.3mf", n))

    for name, mesh in parts:
        path = "%s/%s.stl" % (args.out_dir, name)
        mesh.save(path, name)
        (x0, y0, z0), (x1, y1, z1) = mesh.bounds()
        print("%-12s %5d triangles  %.1f x %.1f x %.1f mm"
              % (name, len(mesh.tris), x1 - x0, y1 - y0, z1 - z0))
    print("outer size %.1f x %.1f mm, shell %.1f mm tall, inner height %.1f mm"
          % (OUTER_W, OUTER_D, SHELL_H, INNER_H))
    print("display top %.1f mm above the floor, Click board %.1f mm (terminals %.1f below it), "
          "screen panel sunk %.1f mm"
          % (DISPLAY_TOP, CLICK_BOARD_Z, CLICK_TERM_STACK, RECESS_DEPTH))


if __name__ == "__main__":
    main()
