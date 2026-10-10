# -*- coding: utf-8 -*-
"""端到端闭环任务: sim_core × detector × guidance → 单发弹道与脱靶量。
los_C → los_Bp 转换: 相机安装为光轴=+xB, 图像上=-zB →
  C 系基 (x_C, y_C, z_C) 在 Bp 下为 (y_B, z_B, x_B), 故 v_Bp = [v_zC, v_xC, v_yC]。"""
import numpy as np
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from sim_core.sim_core import SimCore
from detector.color_blob import Detector
from guidance.pn_pid import Guidance

C2BP = np.array([[0.0, 0.0, 1.0],
                 [1.0, 0.0, 0.0],
                 [0.0, 1.0, 0.0]])   # v_Bp = C2BP @ v_C

# 标称场景: 发射点 (0,0,60) v=30m/s 俯冲, 靶板 (65, 8, 0.5) —
# 可达域内 (带阻力自然射程 ~60-70m), 保留 8m 横向修正需求
P0 = np.array([0.0, 0.0, 60.0])
TARGET = np.array([40.0, 8.0, 0.5])
V_MAG = 30.0
_u = (TARGET - P0) / np.linalg.norm(TARGET - P0)
V0 = V_MAG * _u
_YAW = np.arctan2(_u[1], _u[0])
_PITCH = np.arctan2(-_u[2], np.hypot(_u[0], _u[1]))
RPY0 = [0.0, _PITCH, _YAW]


def default_cfg():
    return {"p0": P0.tolist(), "v0": V0.tolist(), "rpy0": RPY0, "target": TARGET.tolist()}


def run_mission(cfg=None, gui=False, seed=None, verbose=False):
    sim = SimCore(cfg if cfg is not None else default_cfg(), gui=gui, seed=seed)
    det = Detector()
    gnc = Guidance()
    traj = []
    cmd = (0.0, 0.0, 0.0)
    VIS_EVERY = 17               # 500Hz 主循环 / 17 ≈ 30Hz 视觉 (§3 频率解耦)
    while sim.end_reason is None:
        obs = sim.obs()
        if sim.steps % VIS_EVERY == 0:
            frame = sim.get_camera()
            det_out = det.detect(frame)
            if det_out["found"]:
                det_out["los_Bp"] = C2BP @ det_out["los_C"]
            cmd = gnc.update(obs, det_out)
        obs = sim.step(cmd)
        traj.append({"t": obs["t"], "p_W": obs["p_W"], "hit": obs["hit"]})
        if verbose and sim.steps % 250 == 0:
            print(f"t={obs['t']:.2f}s p={np.round(obs['p_W'],1)} cmd={np.round(cmd,3)}")
    obs = sim.obs()
    return {"hit": sim.hit, "miss": sim.miss, "reason": sim.end_reason,
            "t_end": sim.t, "traj": traj}


def p_q2m(q):
    """四元数 → 旋转矩阵 (w,x,y,z)。"""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


if __name__ == "__main__":
    out = run_mission(verbose=True)
    print("=== MISSION RESULT ===")
    print(f"hit={out['hit']}  miss={out['miss']}  reason={out['reason']}  t={out['t_end']:.2f}s")
