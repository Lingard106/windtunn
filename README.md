# 飞镖制导仿真 — 经典控制验证（制导线）

本仓库是 WindTunnelSuite 制导线的**独立精简仓**：只保留视觉/算法侧的仿真、验收与固件移植参考代码。
风洞 CFD 线、Phase 0 占位骨架等协作方内容不在本分支；接口契约与数据契约以本仓文档为准。

## 入口一览

| 启动器 | 验证内容 | 配置文件 | 定位 |
|---|---|---|---|
| **`启动传统控制增强版.cmd`** | 完整刚体物理 + 视觉上靶面估计 + 弹道预测 ZEM/PN + 俯仰/偏航过载 PID + 滚转 P | `guidance_sim/traditional_control/config.json` | **当前推荐主线** |
| `启动v1实跑参数.cmd` | 原 MuJoCo 刚体 + 视线角速率 PNG(N=10) + 三回路自驾 + 舵机/气动 | `guidance_sim/profiles/flight_control_v1_direct_2027.json` | 原方案对照 |
| `启动理想导引仿真.cmd` | 传统 N10 对照，有限过载质点 | `guidance_sim/profiles/ideal_guidance.json` | 几何过面检查 |
| `启动PN专家v2.cmd` | 弹道预测改进 PN（质点模型） | `guidance_sim/pn_expert_v2/config.json` | 导引律实验 |
| `启动参考微调v3.cmd` | 红十字参与靶面微调，可切回仅显示 | `guidance_sim/learning/aim_reference.json` | 标定实验 |
| `采集PN专家数据.cmd` | 批量随机工况专家采样，生成 SL 数据集 | `guidance_sim/learning/config.json` | 数据生产（不训练） |

**参数与成绩不可跨入口混用**：某入口的成绩只对应它自己的 `config.json`；把增强版 PID 数字填进 v1 入口
不能复现增强版结果。

## 关键验收结果

| 对象 | 结果 | 边界 |
|---|---|---|
| 传统控制增强版，联合公差 64 发（seeds 5101–5164） | **64/64 命中**；同视觉原 v1 为 3/64；95% Wilson 94.34%–100%；接触点到靶心均值 20.31 mm / P95 23.12 mm | 质量 ±1% / 初速 ±0.1 m/s / 离架 roll ±1° / Cd ±5%，横风 0；属**换位后静止命中** |
| PN专家v2，29 组真实视觉工况 | 29/29 命中；同初值关控对照 3/29；最大偏靶 1.516 mm | 固定目标；XYZ ±5 cm、侧风 ±1 m/s 等局部扰动 |

均为**模型内**结果，不代表实机性能。完整边界见 [`docs/classical_control_handoff.md`](docs/classical_control_handoff.md) §6。

## 快速开始

首次安装（在本目录打开 PowerShell，使用带 Tkinter 的 Python 3.11）：

```powershell
py -3.11 -m venv guidance_sim/.venv
.\guidance_sim\.venv\Scripts\python.exe -m pip install -r guidance_sim/requirements.txt
```

之后双击任一启动器即可。窗口内支持暂停、重置、调参、切换目标模式与回放。

## 目录结构

```
.
├── README.md                      ← 本文件（总览与导航）
├── docs/
│   ├── classical_control_handoff.md            对接文档: 接口/数据/配置契约、验收边界、上板七步
│   └── control_strategy_and_firmware_porting.md 移植说明: 固件现状核对与 OpenMV/STM32 改法
├── guidance_sim/
│   ├── prototypes/               共用运行与验证代码（渲染/物理/视觉/自驾/回放/诊断）
│   ├── traditional_control/      传统控制增强版（当前推荐主线）
│   ├── pn_expert_v2/             弹道预测专家
│   ├── learning/                 标定参考与 SL 数据采集
│   ├── profiles/                 v1 飞控、理想导引、场地与视觉配置
│   ├── assets/                   运行必需 STL/OBJ 与清单
│   ├── reference/                来源哈希元数据
│   └── README_lite.md            运行手册（逐项参数与诊断结论）
├── firmware_examples/            OpenMV / STM32 移植参考代码
├── devlog/                       开发日志（按日）
└── 启动*.cmd / 采集PN专家数据.cmd  六个入口
```

## 文档索引

- **要接这套仿真** → [`docs/classical_control_handoff.md`](docs/classical_control_handoff.md)：坐标符号、频率预算、观测量与输出边界、通信协议、31 维数据契约、验收边界、上板七步。
- **要上板移植** → [`docs/control_strategy_and_firmware_porting.md`](docs/control_strategy_and_firmware_porting.md)：固件现状逐项核对、OpenMV/STM32 改动清单与示例代码。
- **要跑、要调参** → [`guidance_sim/README_lite.md`](guidance_sim/README_lite.md)：全部入口的运行手册与逐项诊断结论。

## 数据与 Git 边界

- 入库：源码、配置、文档、`guidance_sim/assets/` 的 STL/OBJ 运行资产、`firmware_examples/`。
- 不入库：`guidance_sim/outputs/`（验收报告、截图、NPZ 数据集）、`.venv/`、`__pycache__/`、`local_originals/`（约 1.1 GB 原始 CAD）。
- `assets/` 之所以对 `*.stl` / `*.obj` 破例进库，是因为它们是**运行输入**而非可再生产物。

## 已知边界

气动与舵效为工程估计；几何接触命中不等同官方裁判识别；导航、相机内外参与时延、IMU 安装标度、
AHRS 飞行精度、HIL 与末端持续移动均未验收。**仿真成绩不能解释为实机命中率。**
