#!/usr/bin/env python3
"""
trace_badge.py - trace a badge image (e.g. clover.png) into outlines that
tools/make_case.py embosses on the lid.

    pip install pillow
    python3 tools/trace_badge.py clover.png

It writes tools/badge_data.py with two lists of polygons, normalised to a unit
box centred on (0, 0):

    BADGE_BACK  - the triangle (or whatever sits behind), embossed lower
    BADGE_FRONT - the clover itself, embossed higher

make_case.py picks that file up automatically; without it, it falls back to a
clover drawn from curves. The image itself is NOT in the repository (car maker
logos are trademarks), so neither is the traced data.

How the two layers are separated: coloured (green) pixels and the outline right
around them become the FRONT, grey pixels further away become the BACK, which
is then filled, so the triangle comes out solid like the real badge.

Options:
  --out PATH       where to write the data (default tools/badge_data.py)
  --epsilon PX     outline simplification, bigger = fewer points (default 1.2)
  --min-area PX    ignore specks smaller than this (default 120)
"""
import argparse
import sys
from collections import deque


def masks(im, dilate=5):
    """Return (front, back) boolean grids: coloured parts + their outline, and
    the remaining grey parts."""
    w, h = im.size
    px = im.load()
    colour, grey = [], []
    for y in range(h):
        rowc, rowg = [], []
        for x in range(w):
            r, g, b, a = px[x, y]
            if a < 40 or (r > 215 and g > 215 and b > 215):
                rowc.append(False); rowg.append(False); continue
            is_grey = abs(r - g) < 24 and abs(g - b) < 24
            rowc.append(not is_grey)
            rowg.append(is_grey)
        colour.append(rowc); grey.append(rowg)

    near = [row[:] for row in colour]                  # colour dilated by `dilate`
    for _ in range(dilate):
        prev = [row[:] for row in near]
        for y in range(h):
            for x in range(w):
                if prev[y][x]:
                    continue
                if ((x and prev[y][x - 1]) or (x + 1 < w and prev[y][x + 1]) or
                        (y and prev[y - 1][x]) or (y + 1 < h and prev[y + 1][x])):
                    near[y][x] = True
    front = [[colour[y][x] or (grey[y][x] and near[y][x]) for x in range(w)] for y in range(h)]
    back = [[grey[y][x] and not front[y][x] for x in range(w)] for y in range(h)]
    return front, back


def components(mask, min_area):
    """Split a mask into connected components (4-connectivity)."""
    h, w = len(mask), len(mask[0])
    seen = [[False] * w for _ in range(h)]
    out = []
    for y in range(h):
        for x in range(w):
            if not mask[y][x] or seen[y][x]:
                continue
            q, cells = deque([(x, y)]), []
            seen[y][x] = True
            while q:
                cx, cy = q.popleft()
                cells.append((cx, cy))
                for nx, ny in ((cx-1, cy), (cx+1, cy), (cx, cy-1), (cx, cy+1)):
                    if 0 <= nx < w and 0 <= ny < h and mask[ny][nx] and not seen[ny][nx]:
                        seen[ny][nx] = True
                        q.append((nx, ny))
            if len(cells) >= min_area:
                grid = [[False] * w for _ in range(h)]
                for cx, cy in cells:
                    grid[cy][cx] = True
                out.append(grid)
    return out


def trace(mask):
    """Moore-neighbour boundary tracing of one component; returns pixel ring."""
    h, w = len(mask), len(mask[0])
    start = None
    for y in range(h):
        for x in range(w):
            if mask[y][x]:
                start = (x, y); break
        if start:
            break
    nb = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
    contour = [start]
    cur, d = start, 6
    for _ in range(4 * w * h):
        found = False
        for k in range(8):
            i = (d + 6 + k) % 8                        # start from "back and left"
            nx, ny = cur[0] + nb[i][0], cur[1] + nb[i][1]
            if 0 <= nx < w and 0 <= ny < h and mask[ny][nx]:
                cur, d, found = (nx, ny), i, True
                contour.append(cur)
                break
        if not found or (len(contour) > 2 and cur == start):
            break
    return contour


def simplify(points, eps):
    """Douglas-Peucker."""
    if len(points) < 3:
        return points
    a, b = points[0], points[-1]
    dx, dy = b[0] - a[0], b[1] - a[1]
    norm = (dx * dx + dy * dy) ** 0.5 or 1.0
    worst, idx = 0.0, 0
    for i, p in enumerate(points[1:-1], 1):
        d = abs(dy * (p[0] - a[0]) - dx * (p[1] - a[1])) / norm
        if d > worst:
            worst, idx = d, i
    if worst <= eps:
        return [a, b]
    return simplify(points[:idx + 1], eps)[:-1] + simplify(points[idx:], eps)


def simplify_ring(ring, eps):
    """Douglas-Peucker on a CLOSED ring. Run directly it would collapse: its
    first and last point coincide, so every point sits on that degenerate line.
    Split the ring at the point farthest from the start and simplify each half."""
    if len(ring) > 2 and ring[0] == ring[-1]:
        ring = ring[:-1]
    if len(ring) < 4:
        return ring
    a = ring[0]
    far = max(range(len(ring)), key=lambda i: (ring[i][0] - a[0]) ** 2 + (ring[i][1] - a[1]) ** 2)
    first = simplify(ring[:far + 1], eps)
    second = simplify(ring[far:] + [a], eps)
    return first[:-1] + second[:-1]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image")
    ap.add_argument("--out", default="tools/badge_data.py")
    ap.add_argument("--epsilon", type=float, default=1.2)
    ap.add_argument("--min-area", type=int, default=120)
    args = ap.parse_args()

    from PIL import Image
    im = Image.open(args.image).convert("RGBA")
    front_mask, back_mask = masks(im)

    layers = {}
    for name, mask in (("FRONT", front_mask), ("BACK", back_mask)):
        polys = []
        for comp in components(mask, args.min_area):
            ring = simplify_ring(trace(comp), args.epsilon)
            if len(ring) >= 3:
                polys.append(ring)
        layers[name] = polys
        print("%-5s %d shape(s), %d points total"
              % (name, len(polys), sum(len(p) for p in polys)))
    if not layers["FRONT"]:
        sys.exit("nothing traced - check the image colours")

    pts = [p for polys in layers.values() for poly in polys for p in poly]
    x0, x1 = min(p[0] for p in pts), max(p[0] for p in pts)
    y0, y1 = min(p[1] for p in pts), max(p[1] for p in pts)
    scale = 1.0 / max(x1 - x0, y1 - y0)
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0

    def norm(poly):                                     # image y runs down, model y up
        return [(round((x - cx) * scale, 4), round((cy - y) * scale, 4)) for x, y in poly]

    with open(args.out, "w") as f:
        f.write('"""Badge outlines traced from %s by tools/trace_badge.py.\n'
                'Normalised to a unit box centred on (0, 0). NOT tracked in git."""\n\n'
                % args.image)
        for name in ("BACK", "FRONT"):
            f.write("BADGE_%s = [\n" % name)
            for poly in layers[name]:
                f.write("    %r,\n" % (norm(poly),))
            f.write("]\n\n")
    print("wrote %s (aspect %.2f x %.2f)" % (args.out, (x1 - x0) * scale, (y1 - y0) * scale))


if __name__ == "__main__":
    main()
