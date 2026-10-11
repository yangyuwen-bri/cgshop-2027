"""Evaluate solve_yomi over all example instances.

Writes results/yomi.json (template format) and results/yomi_solutions.zip.
Every solution must pass check_for_errors; any failure aborts loudly.
"""
import datetime
import glob
import json
import os
import sys
import time

from cgshop2027_pyutils.instance_database import InstanceDatabase
from cgshop2027_pyutils.verify import SolutionValidator
from cgshop2027_pyutils.zip import ZipWriter

from solve_yomi import solve_instance


def main(instances_dir: str, out_dir: str) -> int:
    os.makedirs(out_dir, exist_ok=True)
    files = sorted(glob.glob(os.path.join(instances_dir, "**", "*.instance.json"),
                             recursive=True))
    print(f"{len(files)} instances")
    entries = []
    t0 = time.time()
    zip_path = os.path.join(out_dir, "yomi_solutions.zip")
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
            ok = not errors
            entries.append({
                "instance_uid": uid,
                "makespan": sol.max_tour_length,
                "notes": f"solve {dt:.1f}s",
                "solution_file": "results/yomi_solutions.zip",
                "verified": ok,
            })
            writer.add_solution(sol)
            status = "OK " if ok else "FAIL"
            print(f"[{status}] {uid}: makespan={sol.max_tour_length} "
                  f"({dt:.1f}s)" + (f" errors={errors}" if errors else ""))
    report = {
        "agent": "yomi",
        "instances": entries,
        "method": ("recursive-bisection partition into k zones (exact sweep-length "
                   "cut scoring, coarse-to-fine) + tightened boustrophedon sweep "
                   "per zone (skip empty bands, per-row x-tightening, shorter of "
                   "x/y orientations)"),
        "updated_at": datetime.datetime.now(datetime.timezone.utc).strftime(
            "%Y-%m-%dT%H:%M:%SZ"),
    }
    json_path = os.path.join(out_dir, "yomi.json")
    with open(json_path, "w") as f:
        json.dump(report, f, indent=1)
    n_fail = sum(1 for e in entries if not e["verified"])
    print(f"\n{len(entries)} instances, {n_fail} failed, "
          f"total {time.time()-t0:.1f}s")
    print(f"wrote {json_path} and {zip_path}")
    return 1 if n_fail else 0


if __name__ == "__main__":
    instances_dir = sys.argv[1] if len(sys.argv) > 1 else "instances"
    out_dir = sys.argv[2] if len(sys.argv) > 2 else "results"
    sys.exit(main(instances_dir, out_dir))
