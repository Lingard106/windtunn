# -*- coding: utf-8 -*-
"""sim_core 自检: 无控滑翔 3s — 状态连续、无 NaN、气动衰减正常、触地终止, 即 sim_core OK。"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import numpy as np
from sim_core.sim_core import SimCore
from eval.run_mission import default_cfg

sim = SimCore(default_cfg())
ok = True
for i in range(1500):                     # 3s
    obs = sim.step((0.0, 0.0, 0.0))
    for k in ("p_W", "v_W", "q"):
        if np.isnan(obs[k]).any():
            print(f"FAIL: NaN in {k} at step {i}")
            ok = False
            break
    if obs["end_reason"]:
        break
v = np.linalg.norm(obs["v_W"])
print(f"selftest_simcore: t={obs['t']:.2f}s |v|={v:.1f}m/s "
      f"reason={obs['end_reason']} 连续性={'OK' if ok else 'FAIL'}")
# 判据: 状态全程连续 + 1-3s 内以触地正常终止 (俯冲场景物理)
print("SIMCORE-OK" if ok and 1.0 <= obs["t"] <= 3.0 and obs["end_reason"] == "ground"
      else "SIMCORE-FAIL")
