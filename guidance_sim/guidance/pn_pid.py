# -*- coding: utf-8 -*-
"""占位制导律 — 追踪式比例控制 (Phase 0)。
视线误差 (Bp 系) → (de, da, dr) 舵令。滚转通道保持 0。
接口对齐 dev_spec §4.5; 符号约定经端到端命中验证 (Phase 0 的目的之一)。"""
import numpy as np

KP_Y = 0.6     # 偏航通道: 视线横向误差 → 舵
KP_Z = 0.6     # 俯仰通道
KD = 0.15      # 角速度阻尼 (弹体固有 Cmq 已含阻尼, 制导侧只做小幅修正)
MAX_CMD = 0.30


class Guidance:
    def __init__(self, **params):
        self.kp_y = params.get("kp_y", KP_Y)
        self.kp_z = params.get("kp_z", KP_Z)
        self.kd = params.get("kd", KD)
        self.lost_frames = 0

    def update(self, obs, detection, dt_vis=1 / 30) -> tuple:
        """返回 (de, da, dr) 弧度。detection 需含 'los_Bp' (Bp 系视线单位向量,
        由主循环从 los_C 转换; 转换矩阵见 run_mission.py)。"""
        omega = obs["omega_Bp"]         # 体轴角速度 (阻尼反馈)
        if detection is not None and detection.get("found") \
                and detection.get("los_Bp") is not None:
            self.lost_frames = 0
            e = np.asarray(detection["los_Bp"], dtype=float)
            e_y, e_z = e[1], e[2]
        else:
            self.lost_frames += 1
            return (0.0, 0.0, 0.0)     # 丢帧: 舵回中 (Phase 0 策略)

        # 占位气动符号: de>0 = 低头 / dr>0 = 左偏 (端到端验证确定);
        # Phase 0 策略: 横向修正为主, 俯仰通道只留阻尼 (追踪俯冲会损失射程)
        dr = +self.kp_y * e_y - self.kd * omega[2]
        de = -self.kd * omega[1]
        da = 0.0
        return (np.clip(de, -MAX_CMD, MAX_CMD), da,
                np.clip(dr, -MAX_CMD, MAX_CMD))
