# WindTunnelSuite — 双线套件: 桌面风洞 CFD + 制导闭环全仿真

**新读者导读** (按身份选线):
- **视觉/算法侧 (制导闭环项目)** → 先读 [制导闭环全仿真线](#制导闭环全仿真线-2026-10-08-起) 一节;
- 风洞 CFD 线 → 从 [风洞 CFD 线](#风洞-cfd-线) 往下读。

风洞 ([WindTunnel3D](WindTunnel3D/README.md)) 与其供风风机 ([FanSim3D](FanSim3D/)) 的联合工程,
自 2026-10-08 起扩展承载**制导闭环全仿真**项目 (MuJoCo 闭环 + CFD 侧气动数据集)。
各项目同级并列, 相互以「套件相对定位」引用 — 整个 `WindTunnelSuite/` 文件夹
挪到任何盘任何路径都可直接使用, 无需改代码。

```
WindTunnelSuite/
├── README.md            ← 本文件 (套件总览)
├── docs/                跨线文档 (制导对接 spec / 控制路线 / 数据 vault 手册)
├── guidance_sim/        制导线: 闭环仿真 (MuJoCo 环境 + 识别/制导/评估)
├── openfoam/            OpenFOAM case: windtunnel-base / windtunnel-mrf / airframe-sweep (制导线气动库)
├── WindTunnel3D/        风洞: 1D 设计 → MATLAB 体素化 → FluidX3D GPU LBM → 可视化/报告
│                        (共享工具 read_fluidx3d_vtk.m 在其根目录, FanSim3D 经定位器引用)
└── FanSim3D/            风机: 1D 性能模型 → 几何/体素化 → FluidX3D → 可视化
```

## 制导闭环全仿真线 (2026-10-08 起)

**视觉/算法侧同学从这里开始。** 闭环仿真 (目标识别 + 制导律, 500 Hz 主循环 / 30 Hz 视觉)
在 MuJoCo 环境里跑; CFD 侧 (仿真数据集) 交付气动数据库 `aero_table.csv` + 查表参考实现 +
参数化几何。文档链按序读:

| 顺序 | 文档 | 内容 |
|---|---|---|
| 1 | [docs/guidance_loop_dev_spec.md](docs/guidance_loop_dev_spec.md) | **对接主文档 (v0.7)**: 接口规范 / 坐标系约定 / 里程碑与验收 |
| 2 | [docs/control_approach_roadmap.md](docs/control_approach_roadmap.md) | 控制方案路线: B (PN+PID 参数枚举) 主线保底 / A (swarm 端到端 RL) M4+ 增益蒸馏 |
| 3 | [docs/mujoco_alignment_questions.md](docs/mujoco_alignment_questions.md) | MuJoCo 切换对齐问题清单 (待算法侧逐条回复) |
| 4 | [guidance_sim/README.md](guidance_sim/README.md) | 仓库代码: sim_core / detector / guidance / eval (Phase 0 全链路跑通) |
| 5 | [openfoam/airframe-sweep/README.md](openfoam/airframe-sweep/README.md) | CFD 侧工作区: 参数化几何生成器 + STL/MJCF 产物 + 数据集登记 |

- **环境**: Python venv + MuJoCo (外部仿真环境统一 MuJoCo, spec v0.7), 自举步骤见
  guidance_sim/README.md; CFD 交付物 (aero_table / lookup_aero) 纯 numpy, 引擎无关。
- **CFD 侧已交付并验证**: `openfoam/airframe-sweep/geometry/glider.mjcf` 经 MuJoCo 3.16
  实测加载 (惯量逐位一致 + 3s 自由落体解析解吻合); `guidance_sim/aero/lookup_aero.py`
  自测 6/6。待冻结会后扫气出 `aero_table.csv`。

## 风洞 CFD 线

风洞-风机耦合管线 (FluidX3D GPU LBM)、OpenFOAM CPU 稳态管线、数据分层 (wt 工具) —
以下各节均为风洞线内容。

## 路径约定 (2026-09-30 套件化)

- **无盘符硬编码**: 跨项目引用一律 `fileparts(mfilename('fullpath'))` 相对推导
  (`fan_params.m` 推导 `../WindTunnel3D`, `export_fluidx3d_fan.m` 推导 `../FanSim3D`)。
- 环境变量可覆盖: `WT3D_DIR` (风洞根) / `FAN3D_DIR` (风机根)。
- 所有编译 bat 用 `%~dp0` 相对定位, 在任何 cwd 下双击均可编译。
- `D:\WindTunnel3D` / `D:\FanSim3D` 旧位置已清空 (仅剩会话锁定的空壳, 可删)。

## 数据分层 (2026-10-14, wt 工具)

本地仓库只留**运算端** (代码/编译物/raw 输入); 大体积运行数据 (vtk 快照海, ~110G+) 归档到
NAS vault (`WT3D_VAULT`), 本地 scratch 按需 pull 恢复。求解/渲染只读写本地盘, NAS 只做
归档与分发 — 完整守则与迁移手册见 [docs/data-vault.md](docs/data-vault.md)。

```bash
bash wt/wt.sh doctor            # 体检 (NAS 挂载 + wt/wt.conf 配好即绿灯)
bash wt/wt.sh push vtk --hash   # 跑完归档: 本地 fluidx3d/vtk → vault runs/
bash wt/wt.sh pull vtk_<ts>     # 回看: vault → 本地原路径
```

## 数据流 (风机-风洞耦合, ④号管线)

```
FanSim3D 1D 选型 (fan_params/fan_performance/fan_sweep)
   → WindTunnel3D: export_fluidx3d_fan   (体素化 CSG: 大气+y→增压箱→钟形口→吸力眼
     →叶轮绕y轴(xoz平面)→蜗壳xz螺旋→底部切向出口+x→渐扩短管→前置扩散段→…→试验段)
   → build_windtunnel_fan.bat → fluidx3d\windtunnel_fan.exe
   → explore_gpu / gpu_mov / whole_duct_mov / gpu_report
```

风机朝向 (2026-09-30 曼波定): **风洞工作方向=x | 抽风口面朝 +y (气体 -y 进入) |
叶轮绕 y 轴在 xoz 平面旋转 | 蜗壳切向出口沿旋转切线 +x 喷入风洞**。
旋向配对: 蜗壳螺旋逆时针生长 ↔ ω<0 (绕 +y); `WT_SPIN=-1` 整机 z 镜像。

## 快速开始

```matlab
% 风机耦合管线 (WindTunnel3D 内)
export_fluidx3d_fan        % 体素化 + 生成 cpp/bat (WT_QUICK=1 冒烟档)
!build_windtunnel_fan.bat  % 或命令行双击
fluidx3d\windtunnel_fan.exe
explore_gpu                % 交互浏览; whole_duct_mov 出全涵道视频

% 风机单机管线 (FanSim3D 内)
export_fan_fluidx3d
!build_fan.bat
fluidx3d\windfan.exe
explore_fan / fan_mov
```

## 关键开关与注意事项

| 开关 | 作用 |
|---|---|
| `WT_VTK` | `u`(默认)=快照只写 u 场 (~200MB/帧); `all`=u+rho+F (~600MB/帧, F 供表面着色) |
| `WT_QUICK=1` | 冒烟档 (6mm/4000 步) |
| `WT_DX` / `WT_STEPS` | 晶格/步数覆盖 |
| `WT_SPIN=-1` | 风机整机 z 镜像反转 |
| `WT_XTRIM=1` | 风机域定量模式 (隧道折入背压) |

**[2026-09-30 P0 修复] 快照镜像损坏**: FluidX3D 的 `update_fields()` 在本配置下对
rho/u 镜像的重构是坏的 (rho 0.61~2.02、u 低 ~8 倍)。三套求解器模板已在快照前移除该调用
— 主内核每步维护的镜像直接写盘即为真实流场。**此修复之前导出的瞬时 u/rho 快照一律不可信**
(时间时均 mean_u.vtk 与 fan_diag.csv 诊断不受影响)。旧损坏快照已清除。

## OpenFOAM CPU 稳态管线 (openfoam/, 2026-10-04 起)

套件顶层 `openfoam/` 双 case，跑在 WSL2 Ubuntu-24.04：

| case | 内容 | 求解器 | 状态 |
|---|---|---|---|
| `windtunnel-base/` | 五段风洞 blockMesh 57600 格 | simpleFoam (v1912-deb) | ✅ 已验证：入口 16.3 m/s → 试验段 22.00 m/s (= FluidX3D 交叉验证) |
| `windtunnel-mrf/` | 风机+蜗壳+风洞全域 (skin.stl 封闭 skin + impeller.stl)，MRF 叶轮 960rpm | foamRun v12 (见下) | 网格就绪 (666k 格 / 0.689 m³ / 非正交 max 51°)，求解段进行中 |

- **生成器**: `WindTunnel3D/openfoam/gen_openfoam_mrf.m` — 从 `fan_params/fan_performance/fan_geometry`
  单一真源复刻耦合布局 (蜗壳心 xC=-(Rvol+0.03)、舌部 300°、omega 绕 +y 取负与蜗壳螺旋配对)。
  改几何只动 FanSim3D 参数后重跑生成器，全案可重建。
- **运行环境**: v1912-deb 的 MRF 在 noble glibc-2.39 上必崩 (sha1) → MRF case 用 **openfoam.org v12**
  (`source /opt/openfoam12/etc/bashrc`，与 v1912 共存；基线仍走 v1912 + `FOAM_ETC=/usr/share/openfoam/etc
  WM_PROJECT_DIR=/usr/share/openfoam`)。case 放 `~/openfoam/` 跑，/mnt/d/ 会触发 v1912 sha1 bug。
- **关键校核** (gen 完跑 checkMesh 必查): Total volume ≈ 流体手算值 (~0.7 m³，若 ≈4 m³ = ambient 未删，
  skin 泄漏)；patch 清单 skin_casing/skin_inletCap/skin_outletCap/impeller 全在。
- **边界条件约定**: 增压箱顶 skin_inletCap = 总压 0 (大气)，风洞出口 skin_outletCap = 静压 0，
  风机自吸入驱动；omega 符号与蜗壳螺旋配对 (绕 +y 取负，同 FluidX3D cpp omegaFull)。
- 坑点全集见 devlog/2026-10-04.md (STL 封闭性/转置三连/栅格截断/MRF 格式等)。
