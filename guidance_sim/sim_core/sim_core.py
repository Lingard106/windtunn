# -*- coding: utf-8 -*-
"""SimCore — Phase 0 闭环仿真器 (PyBullet 6DOF + 占位解析气动 + 舵机模型)。

接口冻结于 docs/guidance_loop_dev_spec.md §4.2/§4.3。
Bp = PyBullet link 系 (x 前弹轴, y 左, z 上); 世界系 W = PyBullet 世界 (z 上)。
"""
import numpy as np
import pybullet as p
import pybullet_data

import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from aero.placeholder import aero_forces

DT = 0.002                     # 500 Hz
FIN_JOINTS = ["j_fin_up", "j_fin_down", "j_fin_left", "j_fin_right"]
TARGET = np.array([100.0, 8.0, 1.0])   # 靶板中心
HIT_RADIUS = 1.5
FIN_LIMIT = 0.349              # ±20° rad
FIN_RATE = 5.24                # 300°/s rad/s
FIN_TAU = 0.010                # 一阶滞后


class SimCore:
    def __init__(self, cfg=None, gui=False, seed=None):
        self.cfg = cfg or {}
        self.gui = gui
        self.mode = p.CONNECT_GUI if gui else p.DIRECT
        cid = p.connect(self.mode)
        if seed is not None:
            p.resetSimulation()
        p.setPhysicsEngineParameter(fixedTimeStep=DT, numSolverIterations=50)
        p.setGravity(0, 0, -9.81)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        self.target = np.array(self.cfg.get("target", TARGET), dtype=float)

        urdf = os.path.join(os.path.dirname(__file__), "..", "models", "glider_phase0.urdf")
        start = self.cfg.get("p0", [0.0, 0.0, 40.0])
        orn = p.getQuaternionFromEuler(self.cfg.get("rpy0", [0.0, 0.0, 0.0]))
        self.body = p.loadURDF(urdf, start, orn, useFixedBase=False)
        # 关 PyBullet 内建阻尼 — 速度衰减由 aero_forces 自含, 内建阻尼量级失真 (自由飞诊断实锤)
        p.changeDynamics(self.body, -1, linearDamping=0.0, angularDamping=0.0)
        v0 = self.cfg.get("v0", [25.0, 0.0, 0.0])
        p.resetBaseVelocity(self.body, v0, [0, 0, 0])

        self.fin_idx = [i for name in FIN_JOINTS
                        for i in range(p.getNumJoints(self.body))
                        if p.getJointInfo(self.body, i)[1].decode() == name]
        self.fin_act = {"de": 0.0, "da": 0.0, "dr": 0.0}   # 实际偏角 (含速率限制)
        self.fin_cmd = (0.0, 0.0, 0.0)
        self.t = 0.0
        self.steps = 0
        self.hit = False
        self.miss = None
        self.end_reason = None

        # 靶板 (视觉+命中参照): 红色立板 2x2, 面朝 -x
        self.target_board = p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=p.createVisualShape(
                p.GEOM_BOX, halfExtents=[0.05, 1.0, 1.0],
                rgbaColor=[0.85, 0.05, 0.05, 1]),
            basePosition=[self.target[0], self.target[1], self.target[2]])

    # ---- 主推进 ----
    def step(self, delta_cmd):
        """delta_cmd = [de, da, dr] 弧度 (§4.1 混控约定); 滚转 da Phase 0 不用。"""
        de_c, da_c, dr_c = np.asarray(delta_cmd, dtype=float) * 1.0
        # 舵机模型: 速率限制 (占位主导) + 行程限幅
        for ch, val in (("de", de_c), ("da", da_c), ("dr", dr_c)):
            self.fin_act[ch] += np.clip(val - self.fin_act[ch],
                                        -FIN_RATE * DT, FIN_RATE * DT)
        self.fin_act["de"] = np.clip(self.fin_act["de"], -FIN_LIMIT, FIN_LIMIT)
        self.fin_act["da"] = np.clip(self.fin_act["da"], -FIN_LIMIT, FIN_LIMIT)
        self.fin_act["dr"] = np.clip(self.fin_act["dr"], -FIN_LIMIT, FIN_LIMIT)
        self.fin_cmd = (de_c, da_c, dr_c)
        joint_angle = {"j_fin_up": self.fin_act["de"], "j_fin_down": self.fin_act["de"],
                       "j_fin_left": self.fin_act["dr"], "j_fin_right": self.fin_act["dr"]}
        # 舵面 = 纯视觉道具, 运动学摆位 — 关节马达的反作用矩会踢爆低惯量弹体
        # (自由飞诊断: 20ms 内 ω 冲到 2 rad/s = 马达 force=20 对 1e-4 惯量舵面的后坐力);
        # 舵面的真实气动效应由 aero_forces 解析计算, 两者解耦
        for i, name in zip(self.fin_idx, FIN_JOINTS):
            p.resetJointState(self.body, i, targetValue=joint_angle[name])

        # 气动力: 取弹体速度/角速度到 Bp 系施加
        lin_v, ang_v = p.getBaseVelocity(self.body)
        pos, orn = p.getBasePositionAndOrientation(self.body)
        R_WB = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        V_B = R_WB.T @ np.array(lin_v)
        omega_B = R_WB.T @ np.array(ang_v)
        F_B, M_B = aero_forces(V_B, omega_B,
                               self.fin_act["de"], self.fin_act["da"], self.fin_act["dr"])
        # WORLD_FRAME 下 position 为世界坐标 — 必须传当前质心位置, 传 [0,0,0] 等于
        # 把力挂在世界原点上 (70m 力臂 × 阻力 = 170 N·m 乱矩, Phase 0 首个发散真凶)
        p.applyExternalForce(self.body, -1, R_WB @ F_B, pos, p.WORLD_FRAME)
        p.applyExternalTorque(self.body, -1, R_WB @ M_B, p.WORLD_FRAME)

        p.stepSimulation()
        self.t += DT
        self.steps += 1

        # 命中/终止判定
        pos, _ = p.getBasePositionAndOrientation(self.body)
        d = float(np.linalg.norm(np.array(pos) - self.target))
        omega_norm = float(np.linalg.norm(self.obs()["omega_Bp"]))
        if d < HIT_RADIUS:
            self.hit, self.miss, self.end_reason = True, d, "hit"
        elif pos[2] < 0.5:                      # 质心近地 = 触地 (早于接触爆炸)
            self.miss, self.end_reason = d, "ground"
        elif omega_norm > 50.0:                 # 数值发散保护
            self.miss, self.end_reason = d, "diverge"
        elif self.t > 15.0:
            self.miss, self.end_reason = d, "timeout"
        return self.obs()

    def obs(self):
        pos, orn = p.getBasePositionAndOrientation(self.body)
        lin_v, ang_v = p.getBaseVelocity(self.body)
        R_WB = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        return {
            "t": self.t,
            "p_W": np.array(pos), "v_W": np.array(lin_v),
            "q": np.array(orn), "omega_W": np.array(ang_v),
            "R_WB": R_WB, "omega_Bp": R_WB.T @ np.array(ang_v),
            "fin_act": dict(self.fin_act), "fin_cmd": self.fin_cmd,
            "hit": self.hit, "miss_distance": self.miss,
            "end_reason": self.end_reason,
        }

    # ---- 虚拟相机 (§4.3): 相机固连弹头, 光轴沿 +xB, 图像上方向 -zB ----
    def get_camera(self):
        pos, orn = p.getBasePositionAndOrientation(self.body)
        R_WB = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        fx = fy = 554.25                       # FOV 60°, 640px → f = 320/tan(30°)
        cx, cy = 320.0, 240.0
        view = p.computeViewMatrix(
            cameraEyePosition=pos,
            cameraTargetPosition=np.array(pos) + R_WB[:, 0] * 10.0,  # 沿 +xB 看
            cameraUpVector=-R_WB[:, 2])                              # 图像上 = -zB
        proj = p.computeProjectionMatrixFOV(60.0, 4 / 3, 0.1, 500.0)
        w, h, rgb, _, _ = p.getCameraImage(640, 480, view, proj,
                                           renderer=p.ER_TINY_RENDERER)
        img = np.reshape(rgb, (h, w, 4))[:, :, :3].astype(np.uint8)
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        return {"img": img, "K": K, "T_WC": None, "t": self.t,
                "R_WB": R_WB, "pos_W": np.array(pos)}
