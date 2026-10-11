"""Run the baseline over every example instance and report.

For each instance: solve -> verify with SolutionValidator (instance-derived
precomputation paid once) -> record makespan + errors.
Writes results/baseline_results.json and results/baseline_solutions.zip.
"""

import glob
import json
import os
import sys
import time

from cgshop2027_pyutils.instance_database import InstanceDatabase
from cgshop2027_pyutils.verify import SolutionValidator
from cgshop2027_pyutils.zip import ZipWriter

from baseline import solve_instance


def main(instances_dir: str, out_dir: str) -> int:
    os.makedirs(out_dir, exist_ok=True)
    files = sorted(glob.glob(os.path.join(instances_dir, "**", "*.instance.json"),
                             recursive=True))
    print(f"{len(files)} instances")
    results = []
    t0 = time.time()
    zip_path = os.path.join(out_dir, "baseline_solutions.zip")
    with ZipWriter(zip_path) as writer:
        for path in files:
            db = InstanceDatabase(os.path.dirname(path))
            uid = os.path.basename(path)[: -len(".instance.json")]
            inst = db[uid]
            validator = SolutionValidator(inst)
            t1 = time.time()
            sol = solve_instance(inst)
            errors = validator.check_for_errors(sol)
            dt = time.time() - t1
            results.append({
                "instance_uid": uid,
                "n_cutters": inst.number_of_cutters,
                "makespan": sol.max_tour_length,
                "solve_seconds": round(dt, 2),
                "errors": errors,
            })
            writer.add_solution(sol)
            status = "OK " if not errors else "FAIL"
            print(f"[{status}] {uid}: makespan={sol.max_tour_length} "
                  f"({dt:.1f}s)" + (f" errors={errors}" if errors else ""))
    json_path = os.path.join(out_dir, "baseline_results.json")
    with open(json_path, "w") as f:
        json.dump(results, f, indent=1)
    n_fail = sum(1 for r in results if r["errors"])
    makespans = [r["makespan"] for r in results]
    print(f"\n{len(results)} instances, {n_fail} failed, "
          f"total {time.time()-t0:.1f}s")
    if makespans:
        print(f"makespan: min={min(makespans)} max={max(makespans)}")
    print(f"wrote {json_path} and {zip_path}")
    return 1 if n_fail else 0


if __name__ == "__main__":
    instances_dir = sys.argv[1] if len(sys.argv) > 1 else "instances"
    out_dir = sys.argv[2] if len(sys.argv) > 2 else "results"
    sys.exit(main(instances_dir, out_dir))
