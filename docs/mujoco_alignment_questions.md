# MuJoCo 切换对齐问题清单 (CFD 侧 → 视觉/算法侧)

> 2026-10-14 | 背景: 视觉侧确认环境引擎选用 MuJoCo, 对接文档 `guidance_loop_dev_spec.md` 升 v0.6。
> 本清单列出切换前必须双方确认的事项; 逐条回复后结论回写 spec, 作为 Phase 0 冻结会议议程的一部分。
> CFD 侧负责: 曼波 (气动数据库 aero_table.csv / 查表参考实现 / 几何资产)。

## 1. 变更范围 (最高优先级, 决定其余所有条目)

**MuJoCo 承担什么?**

| 方案 | 内容 | 对 CFD 侧的影响 |
|---|---|---|
| A. 仅渲染 | 6DOF 物理仍在 sim_core (PyBullet), MuJoCo 只做 `render(位姿)→图像` | **零影响** — aero_table.csv / lookup_aero 原样交付 |
| B. 物理+渲染 | 6DOF/舵机/气动全部搬进 MuJoCo (free joint + 每步注入气动力), PyBullet 退役 | 需冻结 §2 注入约定; 查表函数接口不变, 注入逻辑由算法侧移植 |

spec §5.1.1 原架构约束是「6DOF 物理永远在 sim_core」——若选 B, 此约束正式废止并记录。

## 2. 气动力注入约定 (仅方案 B 需要)

1. **施加点**: 气动力/力矩参考点 = **全系统质心**。MuJoCo `xfrc_applied` 作用在 body 质心
   (`xipos`); 若 MJCF 的 body 惯量原点与全系统质心不重合, 力矩需换算 — CFD 侧交付的
   参数 JSON 会给出全系统质心坐标, 两侧用同一个值。
2. **参考系**: 查表输出 F_B/M_B 为机体系 B (x 前弹轴, y 右, z 下, 右手系)。MuJoCo body
   frame 的朝向由 MJCF 建模决定 — 建议 MJCF body 直接按 B 系建 (x 前 y 右 z 下),
   省掉一层旋转变换; 若用 MuJoCo 惯例 (x 前 y 左 z 上), 算法侧自行做 B↔MJCF 转换。
3. **无量纲化**: 系数→力的换算 `F = q̄ · S_ref · C`, `q̄ = ½ρV²`。**S_ref、L_ref、力矩参考点
   必须与 CFD 扫气侧完全一致** (值随 aero_table 元数据交付, 见 §4)。
4. **步长**: 主循环 dt = 2 ms 与 MuJoCo `timestep` 对齐 (`<option timestep="0.002"/>`)。
5. **舵机动力学**: spec §4.2 的一阶滞后 (τ=10ms) + 速率限制 (300°/s) + 行程 (±20°) 模型保留
   在算法侧实现。**注意 Phase 0 排障记录的坑在 MuJoCo 同样存在**: 关节马达的反作用矩会踢爆
   低惯量弹体 — 建议舵面仍用运动学摆位 (`mjData.qpos` 直接设角) 或 `actuatorfrc` 隔离, 不用
   位置伺服马达。
6. **接触/约束**: 弹体飞行中不应有接触; 确认 MuJoCo 场景中靶标/地面不在弹体碰撞组内。

## 3. 几何资产交付 (CFD 侧 → 视觉侧)

CFD 侧参数化几何生成器将交付 (`openfoam/airframe-sweep/`):

| 资产 | 内容 | 用途 |
|---|---|---|
| `body.stl` + `fin_{up,down,left,right}.stl` | 弹体与 4 舵面**独立**网格, 米制, 原点=弹体几何中心, B 系朝向 | MJCF asset / snappyHexMesh |
| `airframe_params.json` | 全部几何参数 + 质量惯量 (分部件与全系统合成) + 无量纲化基准 | MJCF inertial / 查表换算 |
| `glider.mjcf` | MuJoCo 模型骨架 (引用上述 STL, free joint + 4 舵 revolute, 零位) | 直接可加载的起点 |

**问题**: v1 几何 = 占位 URDF 同参数 (平头圆柱 L=1.2m/⌀0.15m + 十字平板尾舵 100×120×3mm,
质量 10kg+4×0.05kg)。真实弹形 (头锥/弹翼) 是否已有图样? 有则给参数, CFD 侧生成器改一行
配置重出; 没有则 Phase 1 按占位几何冻结扫气, 真几何到位后重扫 (管线不变)。

MuJoCo 对 URDF 支持有限 (mesh 须外部文件、inertial 须显式), 故 CFD 侧直接给 MJCF+STL,
不再单独维护 URDF 版 (现占位 `glider_phase0.urdf` 由 MJCF 替代)。

## 4. 气动数据库元数据 (需双方冻结, Phase 0 会议)

1. **无量纲化基准**: `S_ref` (v1 默认弹身迎面积 πR² = 0.01767 m²; 备选: 总舵面积)、
   `L_ref` (v1 默认弹径 0.15m)。力矩参考点 = 全系统质心。**改任一项都必须重发表格**。
2. **系数参考轴**: v1 按稳定轴 (风轴) 惯例 — CD 沿来流、CL 垂直来流 (向上为正)、
   CY 侧向; Cm/Cl/Cn 绕体轴。Cm 正 = 抬头 (航空标准)。CL 正 = 沿 −z_B。
   若视觉侧需要体轴口径, 在扫气前提出。
3. **符号约定**: `de/da/dr` 正方向 (占位实现: +de = 水平舵同向后缘向下 = 低头机动)、
   δ1-δ4 与 de/da/dr 的混控映射 (spec §4.1 已有草案, 会议定稿)。
4. **扫气范围**: 可信域 V∈[15,30] m/s, α∈[−5°,15°] 已写入 spec §6。需补充冻结:
   **β 范围** (侧风 Monte Carlo 用; 建议 ±10°)、**舵偏范围** (±20° 全覆盖或工作区缩窄,
   全覆盖 = 表格行数 ×3)。速度域若 Monte Carlo 要撒到 30 m/s 以外, 提前说, 扫气管线扩围。

## 5. 验收与里程碑影响

- M1-M3 验收指标 (检测率/命中率/CEP) **不变** — 引擎无关。
- 相机标称工作点不变: 640×480 @ 30 Hz, FOV 60°, K 矩阵为仿真真值。
- 视觉侧自行完成 MuJoCo 渲染链路验证 (offscreen 出图 + 分辨率/帧率达标), 并把
  `sim_core/camera.py` 的 Renderer 桥指向 MuJoCo; CFD 侧不参与该步, 但 M1 录制序列
  (`record_nominal.py`) 的场景若因引擎切换改变, 通知 CFD 侧留档对应关系。

## 6. 回复方式

逐条编号回复 (如 "1B, 3 有图样另发, 4.4 β±10°"), 结论由 CFD 侧汇总回写 spec v0.7。
