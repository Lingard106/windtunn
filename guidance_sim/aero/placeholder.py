# -*- coding: utf-8 -*-
"""占位解析气动模型 (Phase 0) — 接口与未来 CFD 查表版一致, 真表到位后整体替换。

物理: 细长弹身阻力 + 十字平板舵升力 (小扰动线化, CL=2*pi*alpha_eff 乘展弦比修正)。
坐标系: Bp = PyBullet link 系 (x 前弹轴, y 左, z 上), 右手系。
所有量在 Bp 系, 力单位 N, 力矩单位 N·m。
"""
import numpy as np

RHO = 1.225          # 海平面密度
S_BODY = np.pi * 0.075 ** 2      # 弹身参考面积
S_FIN = 0.10 * 0.12              # 单片舵面积
L_FIN = 0.50                     # 舵面压心到质心力臂
CL_ALPHA_FIN = 2.0 * np.pi * 0.6 # 平板舵升力线 (小展弦比修正 0.6)
CD0 = 0.25                       # 弹身零阻
K_IND = 0.8                      # 诱导阻力系数
Cmq_DAMP = -12.0                 # 俯仰阻尼 (占位: 强阻尼保证全程稳定, 真表到位后替换)
Cnr_DAMP = -12.0                 # 偏航阻尼
OMEGA_DAMP = 2.0                 # 各向角速度阻尼 N·m/(rad/s) — 数值兜底, 防短周期发散


def aero_forces(V_B, omega_B, de, da, dr):
    """占位气动。V_B 弹体系速度, omega_B 角速度, 舵偏为弧度 (de 同向俯仰, da 差动滚转, dr 偏航)。
    返回 (F_B[3], M_B[3]) — 作用在弹体质心的气动力与力矩。"""
    F = np.zeros(3)
    M = np.zeros(3)
    V = np.linalg.norm(V_B)
    if V < 1e-3:
        return F, M
    qbar = 0.5 * RHO * V * V

    # 弹身轴向阻力 (迎面) + 诱导; α/β 限幅 ±25° (无失速模型, 防奇异区数值炸弹)
    alpha = np.clip(np.arctan2(V_B[2], V_B[0]), -0.44, 0.44)
    beta_c = np.clip(np.arctan2(-V_B[1], V_B[0]), -0.44, 0.44)
    F[0] = -qbar * S_BODY * (CD0 + K_IND * alpha ** 2)

    # 十字舵: 水平对产生 z 向升力与俯仰力矩; 垂直对产生 -y 向力与偏航力矩
    # 有效迎角 = 舵偏 + 当地处气流角 (尾部, 气流角近似全弹攻角)
    cl_p = CL_ALPHA_FIN * (de + alpha)
    cl_y = CL_ALPHA_FIN * (-dr + beta_c)
    Fz_fin = qbar * S_FIN * 2.0 * cl_p          # 两片水平舵合力
    Fy_fin = -qbar * S_FIN * 2.0 * cl_y         # 两片垂直舵合力
    F[2] += Fz_fin
    F[1] += Fy_fin

    # 俯仰/偏航力矩 = 舵升力 × 力臂 (r×F: 尾部升力向上 → M[1] 为正 = 低头恢复)
    M[1] = +Fz_fin * L_FIN + qbar * S_BODY * 1.2 ** 2 * Cmq_DAMP * omega_B[1] / (2 * max(V, 1))
    M[2] = -Fy_fin * L_FIN + qbar * S_BODY * 1.2 ** 2 * Cnr_DAMP * omega_B[2] / (2 * max(V, 1))

    # 滚转: 差动舵 (占位线性) + 各向角速度阻尼兜底
    M[0] = qbar * S_FIN * 0.12 * 1.2 * da * 0.5
    M -= OMEGA_DAMP * omega_B
    return F, M


def lookup_aero(V, alpha, beta, de, da, dr):
    """未来 CFD 查表接口的占位实现 — 与 placeholder 相同物理, 签名对齐 aero_table.csv 维度。
    真表到位后: 本函数改为三线性插值 aero_table.csv。"""
    V_B = np.array([V * np.cos(alpha) * np.cos(beta),
                    V * np.sin(beta),
                    V * np.sin(alpha) * np.cos(beta)])
    return aero_forces(V_B, np.zeros(3), de, da, dr)
