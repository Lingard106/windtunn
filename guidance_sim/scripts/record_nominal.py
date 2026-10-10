# -*- coding: utf-8 -*-
"""录制标称弹道图像序列 (M1 识别模块单元测试的输入)。
输出: records/nominal/ 下的 0000.png + meta.csv (t, u, v, los 真值)。"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import numpy as np
import cv2
from eval.run_mission import run_mission

OUT = os.path.join(os.path.dirname(__file__), "..", "records", "nominal")
os.makedirs(OUT, exist_ok=True)


def project_target(frame, target_W):
    """靶板中心在图像中的真值投影 (仿真器 Known)。"""
    R_WB = frame["R_WB"]
    pos = frame["pos_W"]
    p_rel = np.asarray(target_W) - pos
    # 相机系: z_C=+xB, x_C=y_B, y_C=z_B (见 run_mission C2BP)
    p_C = np.array([p_rel @ R_WB[:, 1], p_rel @ R_WB[:, 2], p_rel @ R_WB[:, 0]])
    K = frame["K"]
    uv = K @ (p_C / p_C[2])
    return uv[0], uv[1], p_C / np.linalg.norm(p_C)


# 靶板中心 = sim_core.TARGET
from sim_core.sim_core import TARGET
out = run_mission()
# 重放一遍抓帧 (run_mission 内部不存图 — 简化: 单独跑一遍带录制的主循环)
import pybullet as p
from sim_core.sim_core import SimCore
from guidance.pn_pid import Guidance
from eval.run_mission import C2BP

sim = SimCore(default_cfg())
gnc = Guidance()
det = Detector()
cmd = (0, 0, 0)
meta = []
idx = 0
while sim.end_reason is None:
    obs = sim.obs()
    if sim.steps % 17 == 0:
        frame = sim.get_camera()
        d = det.detect(frame)
        lb = (C2BP @ d["los_C"]) if d["found"] else None
        cv2.imwrite(os.path.join(OUT, f"{idx:04d}.png"),
                    cv2.cvtColor(frame["img"], cv2.COLOR_RGB2BGR))
        ut, vt, lost = project_target(frame, TARGET)
        meta.append([frame["t"], idx, ut, vt, lost[0], lost[1], lost[2],
                     int(d["found"])])
        cmd = gnc.update(obs, {**d, "los_Bp": lb} if d["found"] else d)
        idx += 1
    obs = sim.step(cmd)
np.savetxt(os.path.join(OUT, "meta.csv"), np.array(meta), delimiter=",",
           header="t,idx,u_true,v_true,los_x,los_y,los_z,found_flag")
print(f"record_nominal: {idx} frames + meta.csv -> {OUT}")
