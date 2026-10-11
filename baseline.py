"""Baseline solver for CG:SHOP 2027 (Multi-Robot Lawn Mowing).

Strategy: ONE moving cutter does a boustrophedon ("lawn-mower") sweep over the
whole region bounding box; the remaining cutters stay parked (single-point
tours, explicitly legal per the problem statement).

Why it is always feasible:
- The verifier's coverage model is: swept = Minkowski sum of all visited
  lattice anchor points with the rasterized cutter (relative to its center).
- Rows of anchors are spaced exactly `h` apart, where `h` is the cutter's cell
  height. Row at y=r covers cell rows [r+cy0, r+cy1]; the next row at y=r+h
  covers [r+h+cy0, r+h+cy1] = [r+cy1+1, ...] -- contiguous, no gaps, for ANY
  cutter shape (the horizontal sweep is continuous, so each row's own union
  fills its strip).
- One extra margin row top and bottom (+2 cells margin left/right) covers the
  region edges.
- Rows come in pairs, so the implicitly-closed tour ends at the same x it
  started at: the closing edge is vertical, hence axis-parallel. All other
  edges alternate horizontal/vertical by construction.

This is deliberately dumb -- it is the "pipeline works" target, not a
competitive solution. Expected quality: terrible makespan. Beat it.
"""

from cgshop2027_pyutils.grid import rasterize, rasterize_ring
from cgshop2027_pyutils.schemas import CGSHOP2027Instance, CGSHOP2027Solution


def solve_instance(instance: CGSHOP2027Instance) -> CGSHOP2027Solution:
    cx, cy = instance.cutter_center
    cutter_cells = rasterize_ring(instance.cutter).translated(-cx, -cy)
    ccx0, ccy0, ccx1, ccy1 = cutter_cells.bounds

    # Row spacing: the thinnest cutter column sets the pace. Row at y=r covers
    # cell rows [r+lo(dx), r+hi(dx)] at relative column dx; spacing s keeps
    # consecutive rows contiguous at EVERY column iff
    # s <= min_dx (hi(dx) - lo(dx) + 1). (For solid cutters this is the full
    # cutter height; for cross-shaped cutters it is the arm thickness.)
    col_ys: dict[int, list[int]] = {}
    for dx, dy in cutter_cells.cells():
        col_ys.setdefault(dx, []).append(dy)
    s = min(max(ys) - min(ys) + 1 for ys in col_ys.values())

    region = rasterize(instance.region_to_cover)
    rx0, ry0, rx1, ry1 = region.bounds

    # Anchor rows: row at y=r covers cell rows [r+ccy0, r+ccy1].
    # K rows starting one spacing below the region, ending at the first row
    # that reaches the region's top cell row (r_last + ccy1 >= ry1).
    import math

    r_start = ry0 - ccy0 - s  # one full extra row below the region
    n_rows = math.ceil((ry1 - ccy1 - r_start) / s) + 1
    rows = [r_start + k * s for k in range(n_rows)]
    if len(rows) % 2 == 1:  # even row count -> closing edge is vertical
        rows.append(rows[-1] + s)

    # Anchors sweep x in [x_lo, x_hi], covering cells [x_lo+ccx0, x_hi+ccx1].
    x_lo = rx0 - ccx0 - 2
    x_hi = rx1 - ccx1 + 2

    xs, ys = [], []
    for i, row in enumerate(rows):
        if i % 2 == 0:  # left -> right
            xs += [x_lo, x_hi]
        else:  # right -> left
            xs += [x_hi, x_lo]
        ys += [row, row]

    tours = [{"x": xs, "y": ys}]
    tours += [{"x": [0], "y": [0]}] * (instance.number_of_cutters - 1)

    return CGSHOP2027Solution(instance_uid=instance.instance_uid, tours=tours)


if __name__ == "__main__":
    import sys

    from cgshop2027_pyutils.io import read_instance
    from cgshop2027_pyutils.verify import SolutionValidator

    path = sys.argv[1]
    inst = read_instance(path)
    sol = solve_instance(inst)
    errors = SolutionValidator(inst).check_for_errors(sol)
    print(f"instance: {inst.instance_uid}")
    print(f"cutters: {inst.number_of_cutters}, makespan: {sol.max_tour_length}")
    print("errors:", errors if errors else "none -- feasible")
