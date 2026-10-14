# airframe-sweep — 弹体气动库 CFD 侧工作区 (M0)

对接文档: `../../docs/guidance_loop_dev_spec.md` (v0.6) · MuJoCo 对齐清单: `../../docs/mujoco_alignment_questions.md`

## 定位

制导闭环全仿真的 CFD 侧交付 (**曼波/仿真数据集**): 参数化几何 → OpenFOAM v12 扫气 →
`guidance_sim/aero/aero_table.csv` + 查表参考实现 `guidance_sim/aero/lookup_aero.py`。

## 结构

| 路径 | 内容 | 状态 |
|---|---|---|
| `gen_airframe.m` | 参数化几何生成器 (MATLAB, flat/cone 头型; STL×5 + inertial + MJCF + 参数 JSON) | ✅ v1 |
| `geometry/` | 生成产物 (B 系 x前y右z下, 米制) | ✅ |
| `case_template/` | OpenFOAM v12 弹体绕流扫气 case 模板 (V/α/β/de/da/dr 长表输出) | ⬜ 待冻结会议后建 |

## 已验证 (2026-10-14)

- 生成器三重自检: 薄片积分惯量 vs 解析 (1.2e-05) / STL 包围盒 / 闭合体积 (1.6e-03)
- cone 模式独立验证 (质心后移 -0.0918, 惯量下降, 体积对拍 PASS)
- **MuJoCo 3.16 真机加载** `geometry/glider.mjcf`: 6 body/5 joint/10.2 kg, 惯量逐位一致;
  3s 自由落体 z = -44.17 m ≈ ½gt² (解析), qacc=9.81 纯重力, 无 NaN
- `lookup_aero.py` 自测 6/6 (网格回读/插值/出域 clamp+warning/力矩符号/CSV 闭环)

## 下一步 (按依赖顺序)

1. **Phase 0 冻结会议** (`mujoco_alignment_questions.md` 六节逐条回复): 变更范围 (MuJoCo 只渲染 vs 承载 6DOF)、S_ref/L_ref/符号约定、β 与舵偏扫气范围 — **先于扫气, 否则白扫**
2. 扫气 case 模板: 复用 openfoam/ 风洞管线经验 (WSL2 Ubuntu-24.04, v12 incompressibleFluid + k-ω SST), 弹体用 `geometry/*.stl` (snappyHexMesh), forces 采样挂全弹面
3. 校验点 V=25, α=4°, δ=0: 查量级 (CL~2π·rad(α) 量级) + Cmα<0 静稳定性
4. 全表扫气 → `aero_table.csv` (spec §4.1 长表) → 与 `lookup_aero.py` 联测
5. 占位替换: `aero/placeholder.py` → `lookup_aero.py` (注意 Bp↔B 系转换, 签名已对齐)
