import sys, os, csv, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import numpy as np
from wireconnect.scene import WireConnectScene, SceneConfig
from wireconnect.task import WireConnectTask

N = int(sys.argv[1]) if len(sys.argv) > 1 else 20
scene = WireConnectScene(SceneConfig(), seed=7)
task = WireConnectTask(scene, verbose=False)
rows = []
t0 = time.time()
for i in range(N):
    res = task.run(randomize=True)
    rows.append({
        "trial": i, "success": int(res.success), "grasp": int(res.grasp_ok),
        "perception": int(res.perception_ok), "fallback": int(res.fallback),
        "max_force": round(res.max_force, 2), "retries": res.retries,
        "lateral_mm": round(res.final_tip_err * 1000, 2), "failure": res.failure,
    })
    print(i, rows[-1], flush=True)
out = os.path.join(os.path.dirname(__file__), "..", "media", "trials.csv")
with open(out, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader(); w.writerows(rows)
n = len(rows)
succ = sum(r["success"] for r in rows)
print(f"\n{N} trials: success {succ}/{n} = {succ/n:.1%}  wall {time.time()-t0:.1f}s")
print("failures:", [r["failure"] for r in rows if not r["success"]])
print("csv:", out)
scene.close()
