# -*- coding: utf-8 -*-
"""CFD 侧气动数据库查表参考实现 (spec §4.1) — 替换 placeholder.py 的真表版。

数据源: aero/aero_table.csv 长表 (一行一个工况, spec §4.1 格式):
    V[m/s], alpha[deg], beta[deg], de[deg], da[deg], dr[deg], CL, CD, Cm, CY, Cl, Cn
文件头 '#' 注释行携带元数据 (key=VALUE), 由 gen 扫气管线写出:
    # S_REF=<m^2> L_REF=<m> MOMENT_REF=<point> RHO=<kg/m^3>

坐标系: 输出 F_B/M_B 为 **spec B 系** (x 前弹轴, y 右, z 下, 右手系, 原点=全系统质心)。
注意: 占位 placeholder.py 用的是 PyBullet link 系 Bp (x前 y左 z上) — 两者差 y/z 翻转,
替换进 sim_core 时由仿真器侧转换 (或经 Phase 0 会议统一口径, 见
docs/mujoco_alignment_questions.md §4)。

口径: 系数为稳定轴 (风轴) — CD 沿来流, CL 垂直来流 (向上/−z_B 为正), CY 沿 ŷ_s;
Cm/Cl/Cn 绕体轴, Cm 正 = 抬头 (航空标准)。出域输入 clamp 到表边界并 warning
(spec §6: 出域插值未标定)。

依赖: 仅 numpy。纯函数, 引擎无关 (PyBullet / MuJoCo 均可直接调用)。
"""
import csv
import warnings
from itertools import product

import numpy as np

# 表文件默认位置: 同目录 aero_table.csv
_TABLE_PATH = None
_TABLE = None


class AeroTable:
    """规则六维气动表 (V × alpha × beta × de × da × dr 完整笛卡尔积, 每维等距)。

    coeffs() 返回 6 系数字典; aero_forces() 返回 B 系力/力矩 (N, N·m)。
    """

    KEYS = ("V", "alpha", "beta", "de", "da", "dr")
    COEFFS = ("CL", "CD", "Cm", "CY", "Cl", "Cn")

    def __init__(self, values, coeff, meta):
        """values: dict key->1D 升序等距数组; coeff: (N,6) 系数矩阵 (按笛卡尔积序);
        meta: 元数据 dict (S_REF/L_REF/RHO/...)。"""
        self.values = {k: np.asarray(v, float) for k, v in values.items()}
        self.coeff = np.asarray(coeff, float)
        self.meta = dict(meta)
        n_exp = 1
        for k in self.KEYS:
            n_exp *= len(self.values[k])
        if self.coeff.shape != (n_exp, len(self.COEFFS)):
            raise ValueError(f"表行数 {self.coeff.shape[0]} != 网格笛卡尔积 {n_exp}")
        for k in self.KEYS:
            d = np.diff(self.values[k])
            if len(d) and (np.any(d <= 0) or np.max(d) - np.min(d) > 1e-9):
                raise ValueError(f"表维度 {k} 须严格升序等距 (三线性插值前提)")

    @classmethod
    def from_csv(cls, path):
        """读长表 CSV: '#' 头注释 (key=VALUE 元数据) + 数据行。"""
        meta, rows = {}, []
        with open(path, newline="", encoding="utf-8-sig") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                if line.startswith("#"):
                    for tok in line.lstrip("#").split():
                        if "=" in tok:
                            k, v = tok.split("=", 1)
                            meta[k] = v
                    continue
                rows.append(line.split(","))
        if not rows:
            raise ValueError(f"{path} 无数据行")
        head = [c.strip().lower() for c in rows[0]]
        expect = [k.lower() for k in cls.KEYS] + [c.lower() for c in cls.COEFFS]
        if head != expect:
            raise ValueError(f"表头 {head} 与 spec §4.1 不符: {expect}")
        data = np.array([[float(x) for x in r] for r in rows[1:]], float)
        values, slices = {}, []
        i0 = 0
        for dim, key in enumerate(cls.KEYS):
            u = np.unique(data[:, dim])
            values[key] = u
            slices.append({v: i for i, v in enumerate(u)})
        coeff = np.zeros((int(np.prod([len(values[k]) for k in cls.KEYS])), len(cls.COEFFS)))
        seen = np.zeros(coeff.shape[0], bool)
        for r in data:
            idx = tuple(slices[d][v] for d, v in enumerate(r[:6]))
            flat = np.ravel_multi_index(idx, [len(values[k]) for k in cls.KEYS])
            coeff[flat, :] = r[6:]
            seen[flat] = True
        if not seen.all():
            raise ValueError(f"{path} 笛卡尔积缺格 {int((~seen).sum())} 个 (三线性插值要求完整)")
        for k in ("S_REF", "L_REF", "RHO"):
            if k in meta:
                meta[k] = float(meta[k])
        return cls(values, coeff, meta)

    # ---------- 底层: 系数插值 ----------

    def _loc(self, key, x):
        """返回 (i0, w): 所在格下界索引与分数权重; 出域 clamp + 一次性 warning。"""
        v = self.values[key]
        lo, hi = v[0], v[-1]
        xc = float(np.clip(x, lo, hi))
        if not (lo - 1e-9 <= x <= hi + 1e-9):
            warnings.warn(f"aero_table 出域: {key}={x} -> clamp {xc} (出域未标定, spec §6)",
                          RuntimeWarning, stacklevel=3)
        i0 = int(np.clip(np.searchsorted(v, xc, "right") - 1, 0, len(v) - 2))
        w = (xc - v[i0]) / (v[i0 + 1] - v[i0])
        return i0, w

    def coeffs(self, V, alpha, beta, de, da, dr):
        """六维三线性插值 → 系数字典 (角度单位 deg, 与表一致)。"""
        loc = [self._loc(k, x) for k, x in zip(self.KEYS, (V, alpha, beta, de, da, dr))]
        dims = [len(self.values[k]) for k in self.KEYS]
        out = np.zeros(len(self.COEFFS))
        for corner in product((0, 1), repeat=6):
            idx = tuple(loc[d][0] + corner[d] for d in range(6))
            w = 1.0
            for d in range(6):
                w *= loc[d][1] if corner[d] else (1.0 - loc[d][1])
            if w == 0.0:
                continue
            flat = np.ravel_multi_index(idx, dims)
            out += w * self.coeff[flat, :]
        return dict(zip(self.COEFFS, out))

    # ---------- 上层: 合成 B 系力/力矩 ----------

    def aero_forces(self, V_B, omega_B=None, de=0.0, da=0.0, dr=0.0, damp=None):
        """V_B: B 系速度 (3,) [m/s]; omega_B: B 系角速度 (3,) [rad/s] (阻尼用)。

        damp: 动导数补偿 dict, 键 'Cmq','Cnr','Clp' (量纲: q̄SL_ref²/(2V)·ω̄ 折算),
        spec §4.5.1 补偿层 — 静扫表不含动导数, 工程值由算法侧标定后传入;
        None = 不加阻尼 (与纯表语义一致)。
        返回 (F_B[3], M_B[3]), 力矩参考点 = 全系统质心 (meta.MOMENT_REF)。
        """
        V_B = np.asarray(V_B, float)
        V = float(np.linalg.norm(V_B))
        S, L = self.meta["S_REF"], self.meta["L_REF"]
        rho = self.meta.get("RHO", 1.225)
        if V < 1e-6:
            return np.zeros(3), np.zeros(3)
        u, vv, w = V_B
        alpha = float(np.degrees(np.arctan2(w, u)))                  # B 系 z 下: +w = 正攻角
        beta = float(np.degrees(np.arctan2(vv, np.hypot(u, w))))
        c = self.coeffs(V, alpha, beta, de, da, dr)
        qS = 0.5 * rho * V * V * S
        # 稳定轴基: x_s=V̂; y_s=ŷ_B 在垂直来流平面投影; z_s=x_s×y_s (z 下)
        xh = V_B / V
        ys = np.array([0.0, 1.0, 0.0]) - np.array([0.0, 1.0, 0.0]).dot(xh) * xh
        ys /= np.linalg.norm(ys)
        zs = np.cross(xh, ys)
        F = qS * (c["CD"] * (-xh) + c["CL"] * (-zs) + c["CY"] * ys)
        M = qS * L * (c["Cl"] * np.array([1.0, 0, 0])
                      + c["Cm"] * np.array([0, 1.0, 0])
                      + c["Cn"] * np.array([0, 0, 1.0]))
        if damp and omega_B is not None and V > 1e-6:
            om = np.asarray(omega_B, float)
            qSL2V = 0.5 * rho * V * S * L * L / (2.0 * V)
            M = M + qSL2V * np.array([damp.get("Clp", 0.0) * om[0],
                                      damp.get("Cmq", 0.0) * om[1],
                                      damp.get("Cnr", 0.0) * om[2]])
        return F, M


def load_table(path=None):
    """加载全局表 (默认同目录 aero_table.csv); 幂等。"""
    global _TABLE, _TABLE_PATH
    import os
    if path is None:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "aero_table.csv")
    if _TABLE is None or _TABLE_PATH != path:
        _TABLE = AeroTable.from_csv(path)
        _TABLE_PATH = path
    return _TABLE


def lookup_aero(V, alpha, beta, de, da, dr, table=None):
    """与 placeholder.lookup_aero 同签名 (V/角度为标量, deg)。

    返回 (F_B, M_B) — **B 系** (x前 y右 z下), 与占位的 Bp 系差 y/z 翻转。
    无 aero_table.csv 时抛 FileNotFoundError — 真表尚未交付前请继续用占位。
    """
    t = table if table is not None else load_table()
    V_B = np.array([V * np.cos(np.radians(alpha)) * np.cos(np.radians(beta)),
                    V * np.sin(np.radians(beta)),
                    V * np.sin(np.radians(alpha)) * np.cos(np.radians(beta))])
    return t.aero_forces(V_B, None, de, da, dr)


if __name__ == "__main__":
    # ---- 自测: 合成解析表验证插值/换算/出域行为 ----
    vs = {
        "V": np.array([15.0, 20.0, 25.0, 30.0]),
        "alpha": np.linspace(-5.0, 15.0, 5),
        "beta": np.linspace(-10.0, 10.0, 3),
        "de": np.linspace(-20.0, 20.0, 3),
        "da": np.linspace(-20.0, 20.0, 3),
        "dr": np.linspace(-20.0, 20.0, 3),
    }
    S_REF, L_REF, RHO = 0.017671458676442587, 0.15, 1.225
    grid = [vs[k] for k in AeroTable.KEYS]
    mesh = np.stack(np.meshgrid(*grid, indexing="ij"), -1)
    Vg, ag, bg, eg, dg_, rg = [mesh[..., i] for i in range(6)]
    ar = np.radians(ag)
    CLg = 2.0 * np.pi * ar + 0.8 * np.radians(eg)      # alpha + 舵效线性
    CDg = 0.25 + 4.0 * ar ** 2
    Cmg = -0.5 * ar + 1.2 * np.radians(eg)             # Cmα<0 静稳定
    Z = np.zeros_like(Vg)
    coeff = np.stack([CLg, CDg, Cmg, Z, Z, Z], -1).reshape(-1, 6)
    meta = {"S_REF": S_REF, "L_REF": L_REF, "RHO": RHO}
    t = AeroTable(vs, coeff, meta)

    # 1) 网格点精确回读
    c = t.coeffs(20.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert abs(c["CL"]) < 1e-12 and abs(c["CD"] - 0.25) < 1e-12 and abs(c["Cm"]) < 1e-12, c
    c = t.coeffs(25.0, 5.0, 0.0, 0.0, 0.0, 0.0)
    assert abs(c["CL"] - 2 * np.pi * np.radians(5)) < 1e-9, c
    print("自测1 网格点精确回读        PASS")

    # 2) 格中心插值 = 邻点均值
    c = t.coeffs(22.5, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert abs(c["CD"] - 0.25) < 1e-12 and abs(c["CL"]) < 1e-12, c
    print("自测2 格中心线性插值        PASS")

    # 3) 出域 clamp + warning
    with warnings.catch_warnings(record=True) as wrec:
        warnings.simplefilter("always")
        c = t.coeffs(50.0, 30.0, 0.0, 0.0, 0.0, 0.0)
        assert len(wrec) >= 2, "应各报一次 V/alpha 出域"
        assert abs(c["CD"] - (0.25 + 4 * np.radians(15) ** 2)) < 1e-9, c
    print("自测3 出域 clamp+warning    PASS")

    # 4) 力换算: alpha=5° 时 CL>0 → F_B z 分量为负 (向上 = -z_B); CD>0 → F x 分量为负 (阻力)
    F, M = t.aero_forces(np.array([25.0 * np.cos(np.radians(5)), 0, 25.0 * np.sin(np.radians(5))]))
    qS = 0.5 * RHO * 25.0 ** 2 * S_REF
    assert F[2] < 0 and F[0] < 0, F
    CL_exact = 2 * np.pi * np.radians(5)
    CD_exact = 0.25 + 4 * np.radians(5) ** 2
    assert abs(np.linalg.norm(F) - qS * np.sqrt(CL_exact ** 2 + CD_exact ** 2)) / qS < 1e-9
    # 5) Cm 正 = 抬头: Cm>0 时 M_B y 分量为正
    c5 = t.coeffs(25.0, 0.0, 0.0, 10.0, 0.0, 0.0)
    assert c5["Cm"] > 0
    F, M = t.aero_forces(np.array([25.0, 0, 0]), None, de=10.0)
    assert M[1] > 0, M
    print("自测4/5 力矩换算与符号约定  PASS")

    # 6) lookup_aero 顶层签名 (需要真表文件, 用合成表写临时 CSV 验证)
    import os
    import tempfile
    tmp = os.path.join(tempfile.gettempdir(), "aero_table_selftest.csv")
    with open(tmp, "w", encoding="utf-8") as f:
        f.write("# S_REF=0.017671458676442587 L_REF=0.15 RHO=1.225\n")
        f.write("V,alpha,beta,de,da,dr,CL,CD,Cm,CY,Cl,Cn\n")
        vals = [vs[k] for k in AeroTable.KEYS]
        mg = np.stack(np.meshgrid(*vals, indexing="ij"), -1)
        flat = mg.reshape(-1, 6)
        for row_V, row_a, row_b, row_e, row_d, row_r in flat:
            f.write("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,0,0,0\n"
                    % (row_V, row_a, row_b, row_e, row_d, row_r,
                       2 * np.pi * np.radians(row_a) + 0.8 * np.radians(row_e),
                       0.25 + 4 * np.radians(row_a) ** 2,
                       -0.5 * np.radians(row_a) + 1.2 * np.radians(row_e)))
    t2 = AeroTable.from_csv(tmp)
    F2, M2 = t2.aero_forces(np.array([25.0, 0, 0]), None, de=10.0)
    assert M2[1] > 0 and F2[0] < 0
    os.remove(tmp)
    print("自测6 CSV 长表读写闭环      PASS")
    print("=== lookup_aero 自测全部通过 ===")
