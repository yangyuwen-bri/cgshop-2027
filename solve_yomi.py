"""Yomi solver v1 for CG:SHOP 2027 (Multi-Robot Lawn Mowing).

Strategy
--------
1. Rasterize the region to a cell set.
2. Recursive-bisection partition into k zones (k <= 3 in all examples):
   every cut (vertical/horizontal grid line, coarse-to-fine) is scored by the
   EXACT sweep length of the resulting zones; the cut minimizing
   max(zone costs) wins. A 2-cutter side is costed at sweep_len/2 * 1.15.
3. Each zone is covered by a tightened boustrophedon sweep:
   - anchor rows spaced s apart (s = cutter's thinnest column, as in baseline);
   - anchor rows whose band holds no zone cells are skipped;
   - each row's x-range is tightened to the zone cells in its band
     (+ baseline's -2/+2 margins);
   - the shorter of x-sweep / y-sweep (transposed) is used.
   Row connectors are staircases (horizontal then vertical); an explicit final
   horizontal segment makes the implicit closing edge vertical. All edges are
   axis-parallel lattice segments by construction; consecutive duplicate
   points are removed.

Coverage argument (extends baseline's): consecutive anchor bands tile the
zone's y-range with no gaps (band height >= s), so every zone cell lies in at
least one kept band; the continuous horizontal sweep over [x_lo_r, x_hi_r]
covers [xmin_r - 2, xmax_r + 2] in that band. Union of zones = region.
"""

import math

import numpy as np

from cgshop2027_pyutils.grid import rasterize, rasterize_ring
from cgshop2027_pyutils.schemas import CGSHOP2027Instance, CGSHOP2027Solution


# --------------------------------------------------------------------------
# cutter parameters
# --------------------------------------------------------------------------

def _cutter_params(instance):
    """Return (s, t, ccx0, ccy0, ccx1, cy1): row spacing, column spacing,
    cutter bounds relative to its center."""
    cx, cy = instance.cutter_center
    cells = rasterize_ring(instance.cutter).translated(-cx, -cy)
    ccx0, ccy0, ccx1, ccy1 = cells.bounds
    col_ys = {}
    row_xs = {}
    for dx, dy in cells.cells():
        col_ys.setdefault(dx, []).append(dy)
        row_xs.setdefault(dy, []).append(dx)
    s = min(max(ys) - min(ys) + 1 for ys in col_ys.values())
    t = min(max(xs) - min(xs) + 1 for xs in row_xs.values())
    return s, t, ccx0, ccy0, ccx1, ccy1


# --------------------------------------------------------------------------
# tightened boustrophedon sweep (along x)
# --------------------------------------------------------------------------

def _sweep_x(mask, ox, oy, s, cx0, cy0, cx1, cy1):
    """Boustrophedon sweep along x over the cells in mask.

    mask[row, col] is the cell at (ox + col, oy + row).
    (cx0, cy0, cx1, cy1) = cutter bounds relative to center.
    Returns (xs, ys) as int lists, or None if mask is empty.
    """
    H, W = mask.shape
    if not mask.any():
        return None
    # per mask-row x extents
    any_row = mask.any(axis=1)
    row_xmin = np.where(any_row, np.argmax(mask, axis=1), W)
    row_xmax = np.where(any_row, W - 1 - np.argmax(mask[:, ::-1], axis=1), -1)

    zy0 = int(oy + np.argmax(any_row))
    zy1 = int(oy + (H - 1 - np.argmax(any_row[::-1])))

    r_start = zy0 - cy0 - s
    n_rows = math.ceil((zy1 - cy1 - r_start) / s) + 1

    xs, ys = [], []

    def add_pt(x, y):
        if not xs or xs[-1] != x or ys[-1] != y:
            xs.append(int(x))
            ys.append(int(y))

    n_kept = 0
    for k in range(n_rows):
        r = r_start + k * s
        lo = max(0, r + cy0 - oy)
        hi = min(H - 1, r + cy1 - oy)
        if lo > hi:
            continue
        sub_min = row_xmin[lo:hi + 1]
        sub_max = row_xmax[lo:hi + 1]
        valid = sub_max >= 0
        if not valid.any():
            continue  # band holds no zone cells: skip
        xmin = int(ox + sub_min[valid].min())
        xmax = int(ox + sub_max[valid].max())
        x_lo = xmin - cx0 - 2
        x_hi = xmax - cx1 + 2
        if n_kept % 2 == 0:  # left -> right
            a, b = x_lo, x_hi
        else:  # right -> left
            a, b = x_hi, x_lo
        if n_kept == 0:
            add_pt(a, r)
        else:
            add_pt(a, ys[-1])  # staircase: horizontal ...
            add_pt(a, r)       # ... then vertical
        add_pt(b, r)
        n_kept += 1

    if n_kept == 0:
        return None
    # explicit closing segment: last point -> (x0, r_last); the implicit
    # closing edge (x0, r_last) -> (x0, r_0) is then vertical.
    add_pt(xs[0], ys[-1])
    if len(xs) > 1 and xs[-1] == xs[0] and ys[-1] == ys[0]:
        xs.pop()
        ys.pop()
    return xs, ys


def _sweep_y(mask, ox, oy, t, cx0, cy0, cx1, cy1):
    """Boustrophedon sweep along y (via transposition)."""
    r = _sweep_x(mask.T.copy(), oy, ox, t, cy0, cx0, cy1, cx1)
    if r is None:
        return None
    xs, ys = r
    return ys, xs


def _tour_length(xs, ys):
    n = len(xs)
    return sum(abs(xs[(i + 1) % n] - xs[i]) + abs(ys[(i + 1) % n] - ys[i])
               for i in range(n))


def _zone_tour(mask, ox, oy, P):
    """Shorter of the x / y sweeps for one zone. Returns (xs, ys)."""
    s, t, cx0, cy0, cx1, cy1 = P
    best = None
    r = _sweep_x(mask, ox, oy, s, cx0, cy0, cx1, cy1)
    if r is not None:
        best = (_tour_length(*r), r)
    r = _sweep_y(mask, ox, oy, t, cx0, cy0, cx1, cy1)
    if r is not None:
        L = _tour_length(*r)
        if best is None or L < best[0]:
            best = (L, r)
    return best[1]


def _zone_sweep_len(mask, ox, oy, P):
    """Exact sweep length of one zone (min over orientations)."""
    s, t, cx0, cy0, cx1, cy1 = P
    best = float("inf")
    r = _sweep_x(mask, ox, oy, s, cx0, cy0, cx1, cy1)
    if r is not None:
        best = min(best, _tour_length(*r))
    r = _sweep_y(mask, ox, oy, t, cx0, cy0, cx1, cy1)
    if r is not None:
        best = min(best, _tour_length(*r))
    return best


# --------------------------------------------------------------------------
# recursive-bisection partition
# --------------------------------------------------------------------------

def _coarse(lo, hi, m=41):
    if hi < lo:
        return []
    if hi - lo + 1 <= m:
        return list(range(lo, hi + 1))
    return sorted({int(round(lo + i * (hi - lo) / (m - 1))) for i in range(m)})


def _apply_cut(mask, ox, oy, cut):
    """Split mask along a grid line. Returns ((am, aox, aoy), (bm, box, boy))
    or None if either side is empty."""
    d, c = cut
    H, W = mask.shape
    if d == "v":
        j = c - ox
        if j <= 0 or j >= W:
            return None
        am, bm = mask[:, :j], mask[:, j:]
        if not am.any() or not bm.any():
            return None
        return (am, ox, oy), (bm, c, oy)
    i = c - oy
    if i <= 0 or i >= H:
        return None
    am, bm = mask[:i, :], mask[i:, :]
    if not am.any() or not bm.any():
        return None
    return (am, ox, oy), (bm, ox, c)


def _bisect(mask, ox, oy, n, P):
    """Split zone into n sub-zones (list of (mask, ox, oy))."""
    if n == 1:
        return [(mask, ox, oy)]
    n1, n2 = n // 2, n - n // 2

    def side_cost(sw_len, m):
        return sw_len if m == 1 else sw_len / m * 1.15

    def cut_score(cut):
        parts = _apply_cut(mask, ox, oy, cut)
        if parts is None:
            return None
        (am, aox, aoy), (bm, box, boy) = parts
        la = _zone_sweep_len(am, aox, aoy, P)
        lb = _zone_sweep_len(bm, box, boy, P)
        s1 = max(side_cost(la, n1), side_cost(lb, n2))
        s2 = max(side_cost(la, n2), side_cost(lb, n1))
        return min(s1, s2)

    H, W = mask.shape
    x0, x1 = ox, ox + W - 1
    y0, y1 = oy, oy + H - 1
    best = (float("inf"), None)  # (score, cut)
    for orient, lo, hi in (("v", x0 + 1, x1), ("h", y0 + 1, y1)):
        coarse = _coarse(lo, hi)
        scored = []
        for c in coarse:
            sc = cut_score((orient, c))
            if sc is not None:
                scored.append((sc, c))
        if not scored:
            continue
        scored.sort()
        # refine around the best coarse cut
        c_star = scored[0][1]
        for c in range(max(lo, c_star - 8), min(hi, c_star + 8) + 1):
            if c in coarse:
                continue
            sc = cut_score((orient, c))
            if sc is not None and sc < best[0]:
                best = (sc, (orient, c))
        if scored[0][0] < best[0]:
            best = (scored[0][0], (orient, scored[0][1]))

    if best[1] is None:  # cannot split (e.g. single cell): keep whole
        return [(mask, ox, oy)]
    (am, aox, aoy), (bm, box, boy) = _apply_cut(mask, ox, oy, best[1])
    # assign cutter counts to the cheaper arrangement
    la = _zone_sweep_len(am, aox, aoy, P)
    lb = _zone_sweep_len(bm, box, boy, P)
    if max(side_cost(la, n1), side_cost(lb, n2)) <= max(side_cost(la, n2), side_cost(lb, n1)):
        return _bisect(am, aox, aoy, n1, P) + _bisect(bm, box, boy, n2, P)
    return _bisect(am, aox, aoy, n2, P) + _bisect(bm, box, boy, n1, P)


# --------------------------------------------------------------------------
# entry point
# --------------------------------------------------------------------------

def solve_instance(instance: CGSHOP2027Instance) -> CGSHOP2027Solution:
    P = _cutter_params(instance)
    region = rasterize(instance.region_to_cover)
    mask = region.mask.copy()
    ox, oy = region.origin
    k = instance.number_of_cutters

    zones = _bisect(mask, ox, oy, k, P) if k > 1 else [(mask, ox, oy)]

    tours = []
    for zm, zox, zoy in zones:
        xs, ys = _zone_tour(zm, zox, zoy, P)
        tours.append({"x": xs, "y": ys})
    while len(tours) < k:  # safety: park unused cutters
        tours.append({"x": [0], "y": [0]})
    return CGSHOP2027Solution(instance_uid=instance.instance_uid, tours=tours[:k])


if __name__ == "__main__":
    import sys
    from cgshop2027_pyutils.instance_database import InstanceDatabase
    from cgshop2027_pyutils.verify import SolutionValidator

    db = InstanceDatabase("instances/test_instances1")
    uid = sys.argv[1] if len(sys.argv) > 1 else "fpg73_100k_1s2"
    inst = db[uid]
    sol = solve_instance(inst)
    errors = SolutionValidator(inst).check_for_errors(sol)
    print(f"instance: {uid}, cutters: {inst.number_of_cutters}, "
          f"makespan: {sol.max_tour_length}")
    print("errors:", errors if errors else "none -- feasible")
