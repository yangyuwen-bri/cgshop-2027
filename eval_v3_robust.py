"""Robust incremental eval for solve_yomi v3.

- Solves each instance, verifies with check_for_errors.
- Saves each solution JSON immediately to results/_v3_sol/<uid>.solution.json
- Appends per-instance result to results/_v3_checkpoint.jsonl
- Resumable: skips UIDs already in the checkpoint.
- At the end, assembles results/yomi.json and results/yomi_solutions.zip.

Usage: python eval_v3_robust.py instances/test_instances1 results
"""
import datetime
import glob
import json
import os
import sys
import time
import zipfile

from cgshop2027_pyutils.instance_database import InstanceDatabase
from cgshop2027_pyutils.verify import SolutionValidator

from solve_yomi import solve_instance


def main(instances_dir: str, out_dir: str) -> int:
    os.makedirs(out_dir, exist_ok=True)
    sol_dir = os.path.join(out_dir, "_v3_sol")
    os.makedirs(sol_dir, exist_ok=True)
    ckpt_path = os.path.join(out_dir, "_v3_checkpoint.jsonl")

    done = {}
    if os.path.exists(ckpt_path):
        with open(ckpt_path) as f:
            for line in f:
                line = line.strip()
                if line:
                    e = json.loads(line)
                    done[e["instance_uid"]] = e
        print(f"resuming: {len(done)} already done")

    files = sorted(glob.glob(os.path.join(instances_dir, "**", "*.instance.json"),
                             recursive=True))
    print(f"{len(files)} instances")
    t0 = time.time()
    n_fail = 0
    n_new = 0
    with open(ckpt_path, "a") as ckpt:
        for path in files:
            db = InstanceDatabase(os.path.dirname(path))
            uid = os.path.basename(path)[: -len(".instance.json")]
            if uid in done:
                e = done[uid]
                if not e["verified"]:
                    n_fail += 1
                continue
            inst = db[uid]
            validator = SolutionValidator(inst)
            t1 = time.time()
            try:
                sol = solve_instance(inst)
                errors = validator.check_for_errors(sol)
            except Exception as ex:  # noqa: BLE001 - never lose progress
                errors = [f"EXCEPTION: {type(ex).__name__}: {ex}"]
                sol = None
            dt = time.time() - t1
            ok = not errors
            # save solution immediately
            if sol is not None:
                sp = os.path.join(sol_dir, f"{uid}.solution.json")
                with open(sp, "w") as f:
                    f.write(sol.model_dump_json())
            e = {
                "instance_uid": uid,
                "makespan": sol.max_tour_length if sol is not None else None,
                "notes": f"solve {dt:.1f}s",
                "solution_file": "results/yomi_solutions.zip",
                "verified": ok,
            }
            ckpt.write(json.dumps(e) + "\n")
            ckpt.flush()
            done[uid] = e
            n_new += 1
            if not ok:
                n_fail += 1
            status = "OK " if ok else "FAIL"
            print(f"[{status}] {uid}: makespan={e['makespan']} "
                  f"({dt:.1f}s)" + (f" errors={errors}" if errors else ""),
                  flush=True)

    # assemble yomi.json + zip from checkpoint + saved solutions
    entries = [done[os.path.basename(p)[: -len(".instance.json")]] for p in files]
    zip_path = os.path.join(out_dir, "yomi_solutions.zip")
    if os.path.exists(zip_path):
        os.remove(zip_path)
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for p in files:
            uid = os.path.basename(p)[: -len(".instance.json")]
            sp = os.path.join(sol_dir, f"{uid}.solution.json")
            if os.path.exists(sp):
                zf.write(sp, f"{uid}.solution.json")
    report = {
        "agent": "yomi",
        "instances": entries,
        "method": ("v3: recursive-bisection partition into k zones (exact sweep-length "
                   "cut scoring, vectorized) + per-zone pick of min{x/y zigzag, "
                   "block-STC, skeleton/maze spine + zigzag mop-up} by exact length"),
        "updated_at": datetime.datetime.now(datetime.timezone.utc).strftime(
            "%Y-%m-%dT%H:%M:%SZ"),
    }
    json_path = os.path.join(out_dir, "yomi.json")
    with open(json_path, "w") as f:
        json.dump(report, f, indent=1)
    print(f"\n{len(entries)} instances, {n_fail} failed, {n_new} newly solved, "
          f"total {time.time()-t0:.1f}s")
    print(f"wrote {json_path} and {zip_path}")
    return 1 if n_fail else 0


if __name__ == "__main__":
    instances_dir = sys.argv[1] if len(sys.argv) > 1 else "instances"
    out_dir = sys.argv[2] if len(sys.argv) > 2 else "results"
    sys.exit(main(instances_dir, out_dir))
