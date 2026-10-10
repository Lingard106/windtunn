# 瞬态旋转方案调研（T6，2026-10-06）

> 结论先行：**推荐两阶段——Phase 1 瞬态 MRF（改动最小，~半天级算力），Phase 2 createNonConformalCouples 真滑移旋转（1-2 天级）**。overset/overlappingMesh 路线在 v12 无教程生态，排除。
> 证据全部来自本机 /opt/openfoam12 教程实测（impeller/propeller/rotor2D/mixerVessel2DMRF），非凭记忆。

## 1. 为什么需要瞬态（两个独立动机）

1. **Q 精度**：MRF 冻结转子 Q=0.61 vs FluidX3D 动叶轮 Q=1.38（试验段 10.9 vs 22.0 m/s）。冻结转子把叶轮区的非定常叶片-蜗壳交互平均掉了，前弯多叶风机（叶片通道仅 7 格）对此极敏感。
2. **转速扫掠稳定性（更紧迫）**：790/1080 冷启动长跑发散、960 唯一收敛——这是 **SIMPLE 稳态解在该网格上的存在性问题**，不是初值问题（warm start 跨转速第 50 步即爆）。瞬态求解器把"无稳态解"转化为"时间精确演化"：即使无稳态解也能给出物理上正确的周期/混沌解。**Phase 1 可能直接解锁转速扫掠能力**（T2 判定 1180 的窘境不再依赖稳态解存在）。

## 2. v12 可用机制盘点（教程实证）

| 机制 | v12 教程 | 关键配置 | 算力/改动 |
|---|---|---|---|
| 瞬态 MRF | mixerVessel2DMRF + PIMPLE | 同款 mesh+MRF；controlDict 放开 ddtSchemes（Euler/backward）+ deltaT 1.5e-4 + PIMPLE nOuterCorrectors 2 + adjustTimeStep | 最小（dict 级改动）；~2-3× 单步成本 |
| 滑移接口真旋转 | **impeller**（blockMesh → `createNonConformalCouples -overwrite` → foamRun） | `constant/dynamicMeshDict`：`mover { type motionSolver; motionSolver solidBody; cellZone rotating; solidBodyMotionFunction rotatingMotion; origin/axis/omega }`；旋转壁 BC = `movingWallVelocity`/`movingWallSlipVelocity` | 中（需网格手术：rotorZone 边界转 couple）；PIMPLE 瞬态 |
| overset/overlapping | **无教程**（tutorials 全树无 overset/overlap 命中） | — | 排除 |
| SRF 单旋转系 | mixerSRF/rotor2DSRF | 整域旋转系——与静止蜗壳+风洞不匹配 | 不适用 |

**关键风险（Phase 2）**：impeller 教程的接口是 blockMesh 规整网格；我们的 case 是 snappy 阶梯网格，rotorZone（topoSet cylinderToCell，r=r2+0.002）边界面在阶梯处切割——`createNonConformalCouples` 本身就是为非共形 couple 设计的，但接口面质量（overlap/泄漏）在阶梯边界上未经验证，需要小样先试。

## 3. 时间步与算力预算（Phase 1，7.9GB WSL / 10 核）

- 叶片通过频率 BPF = 960/60 × ~16 叶 ≈ 256 Hz（周期 3.9 ms；叶数以 fan_params 为准）
- dt 选取：转子区 4mm 格 × 局部 ~20 m/s → Co 限制 dt ≈ 2e-4 s ≈ 20 步/BPF ✓
- 统计收敛：≥40 BPF ≈ 0.16 s ≈ 800 步起步；取 **endTime 2000 瞬态步（0.4 s ≈ 100 BPF）** 与现稳态口径对齐
- 单步成本 ≈ 2-3× SIMPLE 步（2 outer × 压力修正链）≈ 5-8 s/步 → **2000 步 ≈ 3-4.5 h**，内存增量可忽略
- 判定量：Q（outletQ）时均 + 波动幅值（叶片通过周期振荡是健康指标，非发散）；对比稳态 MRF 0.61 与 FluidX3D 1.38 的差距归因

## 4. 实施步骤（Phase 1，不动网格）

1. 从 960 基线 case 派生 `windtunnel-mrf-trans`（copy，勿覆盖稳态正本）
2. controlDict：`ddtSchemes backward`（或先 Euler 稳一点）、`deltaT 1.5e-4`、`adjustTimeStep yes; maxCo 0.8;`、`application foamRun; solver incompressibleFluid` 不变、PIMPLE `nOuterCorrectors 2; nCorrectors 1;`（fvSolution 加 PIMPLE 块，SIMPLE 块保留兼容）
3. 起步策略：从稳态 960 收敛场热启动（latestTime）——先小 rpm 差验证瞬态循环稳定，再回 960 长跑；790/1080 各跑 800 步看是否"发散"变"周期解"（**T2 的扫掠窘境验证实验**）
4. 判定门槛：Q 时均 vs 0.61；Q 波动幅值/均值比；末段是否仍单调漂移（真发散）或周期稳定

## 5. Phase 2 预研要点（不实施）

- 网格手术：topoSet 分 rotorZone → boundary 手术把 zone 边界面提为 couple patch 面 → createNonConformalCouples → mover dict（omega=-960rpm 绕 +y，注意 MRF 经验的负号约定对应关系）
- 小样验证：先用 2D/粗 3D 圆柱接口验证 couple 泄漏，再上真几何
- 预期收益：彻底消除 MRF 平均化误差；叶尖间隙流动真实化；代价=接口质量风险+每步网格更新成本

## 6. 与其他任务的关系

- T2（1180 稳态判定）照常——它是 Phase 1 的对照组；Phase 1 的 790/1080 瞬态实验直接回答"扫掠发散是解存在性还是物理不稳定"
- T3（skew faces 清理）先行有利：skew faces 在瞬态大梯度下危害放大
- T7（3mm 加密）与 Phase 2 正交（加密改背景格，滑移改旋转处理），最终 25 m/s 冲刺可能两者都要
