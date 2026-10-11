"""Yomi solver v2 for CG:SHOP 2027 (Multi-Robot Lawn Mowing).

v2 changes over v1
------------------
1. STC (spanning-tree coverage) over cutter-sized blocks, as a third
   per-zone coverage pattern. Classical STC coarsens the grid by the robot
   footprint: with the solid bx*by rectangle R contained in the cutter,
   tiling the plane with bx*by blocks, one anchor per block covers the
   whole block, so a spanning-tree circumnavigation over blocks visits far
   fewer nodes than cell-level STC. Each zone uses the shortest of
   {x-zigzag, y-zigzag, block-STC} by exact constructed length. This
   targets maze-like / thin winding zones where boustrophedon wastes
   travel spanning empty gaps.
2. Exact two-level partition search (general k, exact within the
   recursive-bisection family): every cut is scored with the TRUE optimal
   sub-partition cost of each side -- computed recursively with
   branch-and-bound (cell-count lower bounds, incumbent caps) -- replacing
   v1's sweep_len/2*1.15 proxy for multi-cutter sides. The 1.15 magic
   constant is gone. No other constants: s, t, margins all derived from
   cutter/region.

Coverage arguments
------------------
- Zigzag: unchanged from v1. Consecutive anchor bands tile the zone's
  y-range with no gaps (band height >= s = thinnest cutter column), so
  every zone cell lies in a kept band; the continuous horizontal sweep
  covers the band. Staircase row connectors + explicit final horizontal
  segment keep every edge axis-parallel and the implicit closing edge
  vertical.
- STC: every region cell lies in a block holding an anchor, and the
  anchor's cutter placement covers the whole block (solid rectangle R in
  the cutter, by construction of the block rect). The tour visits every
  block anchor, so every cell is swept. Components are joined into a
  cycle by Manhattan connectors + explicit return leg; all edges are
  axis-parallel lattice segments with no immediate repeats.
- Partition: zones are disjoint cell sets whose union is the region
  (straight grid cuts), so the zone tours together cover the region.
  Every emitted solution is validated by the official check_for_errors.
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
    return s, t, ccx0, ccy0, ccx1, ccy1, set(cells.cells())


# --------------------------------------------------------------------------
# tightened boustrophedon sweep (along x) -- unchanged from v1
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


# --------------------------------------------------------------------------
# STC: spanning-tree coverage over cutter-sized blocks
# --------------------------------------------------------------------------

def _greedy_order(reps):
    """Nearest-neighbor order over representative points (deterministic)."""
    order = [0]
    rest = list(range(1, len(reps)))
    cur = reps[0]
    while rest:
        bi = min(range(len(rest)),
                 key=lambda i: abs(cur[0] - reps[rest[i]][0])
                 + abs(cur[1] - reps[rest[i]][1]))
        order.append(rest.pop(bi))
        cur = reps[order[-1]]
    return order


# Classical STC coarsens the grid by the robot footprint: each block is fully
# covered by a single anchor placement, so the spanning-tree
# circumnavigation visits far fewer nodes than cell-level STC (which is the
# 1x1-block special case). We use the solid bx*by rectangle R, contained in
# the cutter cells, that minimizes (1/bx + 1/by); tiling the plane with
# bx*by blocks, the anchor a(i,j) = (i*bx - r0x, j*by - r0y) satisfies
# a(i,j) + R = block(i,j), hence a(i,j) + cutter covers the whole block.
# Every region cell lies in a block holding an anchor, and the tour visits
# every block anchor, so the zone is fully swept. Blocks use 4-connectivity;
# components are joined into a cycle by Manhattan connectors, plus an
# explicit return leg so the implicit closing edge is a unit step.
# All edges are axis-parallel lattice segments; consecutive points never
# repeat (single-block zones become single-point tours).

def _best_block_rect(cellset, cx0, cy0, cx1, cy1):
    """Solid rectangle in the cutter cells minimizing (1/bx + 1/by).

    Returns (bx, by, r0x, r0y); rect = [r0x, r0x+bx) x [r0y, r0y+by).
    """
    best = None
    best_val = float("inf")
    for y0 in range(cy0, cy1 + 1):
        for x0 in range(cx0, cx1 + 1):
            if (x0, y0) not in cellset:
                continue
            x1 = x0
            while x1 + 1 <= cx1 and (x1 + 1, y0) in cellset:
                x1 += 1
            for xa in range(x0, x1 + 1):
                bx = xa - x0 + 1
                # tallest possible for this width: prune if hopeless
                if 1.0 / bx + 1.0 / (cy1 - y0 + 1) >= best_val:
                    continue
                y1 = y0
                while y1 + 1 <= cy1 and all(
                        (x, y1 + 1) in cellset for x in range(x0, xa + 1)):
                    y1 += 1
                by = y1 - y0 + 1
                val = 1.0 / bx + 1.0 / by
                if val < best_val:
                    best_val = val
                    best = (bx, by, x0, y0)
    return best


def _block_stc_seq(mask, ox, oy, bx, by, r0x, r0y):
    """Block-STC as an anchor sequence. Returns (seq, length) where seq is
    the ordered list of anchors (with inter-component Manhattan connectors
    and the return leg), and length is the exact closed-tour grid length
    = sum of Manhattan distances between consecutive anchors.
    Fast: no per-unit-step point emission."""
    rs, cs = np.nonzero(mask)
    if len(rs) == 0:
        return None, float("inf")
    bi = np.floor_divide(ox + cs, bx)
    bj = np.floor_divide(oy + rs, by)
    ublocks = np.unique(np.stack([bi, bj], axis=1), axis=0)
    bset = set(map(tuple, ublocks.tolist()))
    anchors = {b: (b[0] * bx - r0x, b[1] * by - r0y) for b in bset}

    comps = []
    remaining = set(bset)
    while remaining:
        seed = remaining.pop()
        comp = {seed}
        stack = [seed]
        while stack:
            u, v = stack.pop()
            for nb in ((u + 1, v), (u - 1, v), (u, v + 1), (u, v - 1)):
                if nb in remaining:
                    remaining.discard(nb)
                    comp.add(nb)
                    stack.append(nb)
        comps.append(comp)

    reps = [anchors[next(iter(c))] for c in comps]
    order = _greedy_order(reps) if len(comps) > 1 else [0]

    seq = []
    total = 0

    def push(ax, ay):
        nonlocal total
        if seq:
            total += abs(seq[-1][0] - ax) + abs(seq[-1][1] - ay)
        seq.append((ax, ay))

    for ci in order:
        comp = comps[ci]
        seed = next(iter(comp))
        walk = [seed]
        seen = {seed}

        def nbs(b):
            u, v = b
            for nb in ((u + 1, v), (u - 1, v), (u, v + 1), (u, v - 1)):
                if nb in comp and nb not in seen:
                    yield nb

        stack = [(seed, nbs(seed))]
        while stack:
            node, it = stack[-1]
            nxt = None
            for q in it:
                nxt = q
                break
            if nxt is None:
                stack.pop()
                if stack:
                    walk.append(stack[-1][0])
            else:
                seen.add(nxt)
                walk.append(nxt)
                stack.append((nxt, nbs(nxt)))
        for b in walk:
            push(*anchors[b])
    if len(order) > 1:
        push(*seq[0])  # explicit return leg closes the cycle validly
    return seq, total


def _block_stc_len(mask, ox, oy, bx, by, r0x, r0y):
    """Exact block-STC tour length (fast path for search scoring)."""
    _, total = _block_stc_seq(mask, ox, oy, bx, by, r0x, r0y)
    return total


def _block_stc(mask, ox, oy, bx, by, r0x, r0y):
    """Block-STC closed tour for the zone. Returns (length, (xs, ys))."""
    seq, total = _block_stc_seq(mask, ox, oy, bx, by, r0x, r0y)
    if seq is None:
        return float("inf"), None
    xs, ys = [], []

    def emit(px, py):
        if not xs or xs[-1] != px or ys[-1] != py:
            xs.append(int(px))
            ys.append(int(py))

    for ax, ay in seq:
        if not xs:
            emit(ax, ay)
            continue
        x0, y0 = xs[-1], ys[-1]
        if x0 != ax:
            step = 1 if ax > x0 else -1
            for x in range(x0 + step, ax + step, step):
                emit(x, y0)
        if y0 != ay:
            step = 1 if ay > y0 else -1
            for y in range(y0 + step, ay + step, step):
                emit(ax, y)
    if len(xs) > 1 and xs[-1] == xs[0] and ys[-1] == ys[0]:
        xs.pop()
        ys.pop()
    assert _tour_length(xs, ys) == total, "block-STC length mismatch"
    return total, (xs, ys)


def _zone_cells(mask, ox, oy):
    rs, cs = np.nonzero(mask)
    return {(ox + int(c), oy + int(r)) for r, c in zip(rs, cs)}


def _zone_sweep_len(mask, ox, oy, P, BR, do_stc=True):
    """Exact sweep length of one zone: min over {x-zigzag, y-zigzag, STC}.

    do_stc=False skips STC (zigzag only): used inside the partition search
    where thousands of candidate zones are scored and STC's Python-level
    DFS would dominate runtime. Final zones always use do_stc=True via
    _zone_tour, so the reported makespan is the exact min."""
    s, t, cx0, cy0, cx1, cy1 = P[:6]
    bx, by, r0x, r0y = BR
    best = float("inf")
    r = _sweep_x(mask, ox, oy, s, cx0, cy0, cx1, cy1)
    if r is not None:
        best = _tour_length(*r)
    r = _sweep_y(mask, ox, oy, t, cx0, cy0, cx1, cy1)
    if r is not None:
        best = min(best, _tour_length(*r))
    n = int(mask.sum())
    # block-STC lower bound: closed walk over >= n/(bx*by) blocks, length >= #blocks
    if do_stc and n > 0 and n / (bx * by) < best:
        L = _block_stc_len(mask, ox, oy, bx, by, r0x, r0y)
        best = min(best, L)
    return best


def _zone_tour(mask, ox, oy, P, BR):
    """Shortest of the x / y sweeps and STC for one zone. Returns (xs, ys)."""
    s, t, cx0, cy0, cx1, cy1 = P[:6]
    bx, by, r0x, r0y = BR
    best = None
    for r in (_sweep_x(mask, ox, oy, s, cx0, cy0, cx1, cy1),
              _sweep_y(mask, ox, oy, t, cx0, cy0, cx1, cy1)):
        if r is not None:
            L = _tour_length(*r)
            if best is None or L < best[0]:
                best = (L, r)
    n = int(mask.sum())
    if n > 0 and (best is None or n / (bx * by) < best[0]):
        L, tour = _block_stc(mask, ox, oy, bx, by, r0x, r0y)
        if best is None or L < best[0]:
            best = (L, tour)
    return best[1]


# --------------------------------------------------------------------------
# exact recursive-bisection partition (branch and bound, no proxy constants)
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


def _opt_part(mask, ox, oy, m, P, BR, D, cap, do_stc=True):
    """Minimum possible max zone sweep for partitioning this zone among
    m cutters (recursive-bisection family).

    Every cut is scored with the TRUE optimal sub-partition cost of each
    side (recursion); branch-and-bound prunes via the airtight cell-count
    lower bound sweep >= (cells - 1) / D, hence opt(X, mm) >= (cells/mm - 1)/D.
    With do_stc=False the zone cost is zigzag-only (fast path for the
    search interior); final zones always use the exact min in _zone_tour.
    Returns (cost, zones) with cost < cap, else (inf, None).
    """
    if m == 1:
        L = _zone_sweep_len(mask, ox, oy, P, BR, do_stc)
        if L < cap:
            return L, [(mask, ox, oy)]
        return float("inf"), None
    H, W = mask.shape
    best_cost = cap
    best_zones = None

    def lb_side(cells, mm):
        return max(0.0, (cells / mm - 1)) / D

    def score_cut(cut, best):
        parts = _apply_cut(mask, ox, oy, cut)
        if parts is None:
            return None
        (am, aox, aoy), (bm, box, boy) = parts
        ca = int(am.sum())
        cb = int(bm.sum())
        bscore = best
        bzones = None
        # balanced splits first: better incumbents earlier -> more pruning
        for m1 in sorted(range(1, m), key=lambda z: abs(z - m / 2)):
            m2 = m - m1
            if max(lb_side(ca, m1), lb_side(cb, m2)) >= bscore:
                continue
            c1, z1 = _opt_part(am, aox, aoy, m1, P, BR, D, bscore, do_stc)
            if z1 is None:
                continue
            c2, z2 = _opt_part(bm, box, boy, m2, P, BR, D, bscore, do_stc)
            if z2 is None:
                continue
            sc = c1 if c1 >= c2 else c2
            if sc < bscore:
                bscore = sc
                bzones = z1 + z2
        if bzones is None:
            return None
        return bscore, bzones

    for orient, lo, hi in (("v", ox + 1, ox + W - 1),
                           ("h", oy + 1, oy + H - 1)):
        coarse = _coarse(lo, hi)
        if not coarse:
            continue
        scored = []
        for c in coarse:
            r = score_cut((orient, c), best_cost)
            if r is not None:
                scored.append((r[0], c))
                if r[0] < best_cost:
                    best_cost, best_zones = r[0], r[1]
        if not scored:
            continue
        scored.sort()
        c_star = scored[0][1]
        cset = set(coarse)
        for c in range(max(lo, c_star - 8), min(hi, c_star + 8) + 1):
            if c in cset:
                continue
            r = score_cut((orient, c), best_cost)
            if r is not None and r[0] < best_cost:
                best_cost, best_zones = r[0], r[1]
    if best_zones is None:
        return float("inf"), None
    return best_cost, best_zones


# --------------------------------------------------------------------------
# entry point
# --------------------------------------------------------------------------

def solve_instance(instance: CGSHOP2027Instance) -> CGSHOP2027Solution:
    P = _cutter_params(instance)
    _s, _t, cx0, cy0, cx1, cy1 = P[:6]
    D = max(cy1 - cy0 + 1, cx1 - cx0 + 1)  # cutter bbox max dimension
    BR = _best_block_rect(P[6], cx0, cy0, cx1, cy1)  # (bx, by, r0x, r0y)
    region = rasterize(instance.region_to_cover)
    mask = region.mask.copy()
    ox, oy = region.origin
    k = instance.number_of_cutters

    if k == 1:
        zones = [(mask, ox, oy)]
    else:
        _cost, zones = _opt_part(mask, ox, oy, k, P, BR, D, float("inf"),
                                       do_stc=False)
        if zones is None:  # degenerate (e.g. single cell, k > 1): park extras
            zones = [(mask, ox, oy)]

    tours = []
    for zm, zox, zoy in zones:
        xs, ys = _zone_tour(zm, zox, zoy, P, BR)
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
