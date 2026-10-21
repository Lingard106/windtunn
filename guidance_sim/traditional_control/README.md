# traditional_control 完整物理增强分区交接

本分区把真实视觉估计与弹道预测导引接到既有过载自驾仪，仍由原 MuJoCo 刚体、被动气动、四舵/PWM/舵机执行。它通过 `PhysicalCandidate` 继承共同物理模型，和[理想 v2](../pn_expert_v2/README.md)的执行模型不同。总导航见[仿真区 README](../README.md)。盘点日期：2026-10-21；本次未运行窗口或验收。

## 文件与调用关系

| 文件 | 责任 |
|---|---|
| [run.py](run.py) | 参数解析、启动窗口、随机公差抽样、真实相机无窗口飞行、配对 v1 比较、汇总与增量保存 |
| [simulation.py](simulation.py) | `PhysicalCandidate`、`CommandAutopilot`、独立配置加载、静态标定视觉估计、专家到机体系请求桥接、固定标称预测模型、状态快照 |
| [vision.py](vision.py) | `ReacquiringTracker` 包装原跟踪器，仅修补“未确认前丢失后返回全图搜索”状态转移 |
| [viewer.py](viewer.py) | `PhysicalTraditionalViewer`，显示实际生效的预测参数与分轴 PID，导出完整方案 |
| [config.json](config.json) | 由 `candidate`、`prediction_model`、`flight_control` 组成的独立方案包 |
| [__init__.py](__init__.py) | 对外导出仿真类与配置加载接口 |

运行还依赖 [prototypes](../prototypes/README.md)、[pn_expert_v2/expert.py](../pn_expert_v2/expert.py)、[profiles](../profiles/README.md)、资产与依赖文件；本目录不是可独立部署的软件包。

## 启动与批量命令

在工作区根目录 PowerShell 执行。根目录的[运行传统控制仿真.cmd](../../运行传统控制仿真.cmd)默认等价于第一条。

```powershell
.\guidance_sim\.venv\Scripts\python.exe guidance_sim/traditional_control/run.py --auto-run

# 现有无窗口配对验收接口；会运行仿真并写输出，本次未执行
.\guidance_sim\.venv\Scripts\python.exe guidance_sim/traditional_control/run.py --headless --target-mode random_moving --randomize match --randomization-seed 91003 --seeds (5101..5164) --compare-v1 --output guidance_sim/outputs/traditional_control_recheck
```

默认目标模式 `random_moving`、目标种子 2027、飞控开启、时长 2.6 s、随机公差 `none`；窗口和无窗口运行均默认不写文件，无窗口结果打印到控制台。`--seeds`、`--compare-v1`、`--output`、`--trace` 必须同时使用 `--headless`，`--trace` 还须显式指定 `--output`；重复种子会被拒绝。`--flight-config` 读取本分区完整方案，`--config` 是数值物性覆盖，两者不能互换。

其他已实现参数包括场地/镖体/发射清单、`--vision-config`、风/偏航/俯仰/初速偏移、`--roll-deg`、IMU 噪声倍率与自动关闭。四种目标模式由共同的 `target_motion.py` 提供。

## 当前配置快照

| 配置节 | 当前值与意义 |
|---|---|
| `candidate` | N=3，合成预测请求上限 0.2 g，响应 0.1 s，变化率 2.5 g/s，更新 0.02 s，预测步长 0.025 s，视觉超时 0.15 s、滤波 0.08 s |
| `prediction_model` | 固定标称质量 0.25 kg、密度 1.225 kg/m³、面积 0.0033587151005579547 m²、Cd=0.35；不从公差后的真实模型回填 |
| `flight_control.controller` | `acceleration_pid`；俯仰/偏航 Kp=80、Ki=100、Kd=0，角速率反馈 0.1 s；滚转 `legacy_p`，原 P=0.1 |
| `flight_control.servo` | 30°限位、300°/s、0.01 s 时间常数、20 ms PWM |
| `flight_control.actuation` | 保留原估计舵效，不等同实测气动能力 |

内层 `flight_control.provenance.description` 留有旧 v1 参数描述；外层已声明其作为历史来源保留。实际 `autopilot_mode`、PID 字段及外层 `candidate` 才是当前生效项，不能只读 provenance 文字判断控制模式。窗口预测页可导出完整 bundle；只复制 PID 数字到 v1 配置不能复现本分区。

随机公差支持 `match`（质量±1%、速度±0.1 m/s、离架 roll±1°、Cd±5%）、`small`（±2%、±0.2 m/s、±2°、±10%）、`large`（±5%、±0.5 m/s、±5°、±20%）。每发独立均匀抽样并保持；质量同比例缩放惯量。`--randomization-components mass speed roll drag` 可选择分量，默认全选。初始 roll 改机体与相机，不改初速度方向或注入自旋。

## 输入、结果与边界

真实图像经绿灯检测、静态灯平面和灯到上靶面偏移得到估计目标；预测器使用理想自身状态、标称物性和零风估计，不获取动态靶真值、随机种子或真实风。用测得的总法向比力作为预测响应状态也是近似。

仅显式 `--headless --output` 时写目录内 `result.json`：计划种子、配置/哈希、逐发参数、接触和过面事件、异常、命中比例及 Wilson 区间。每完成一发即重写报告，`complete` 表示各变体是否齐全。另加 `--trace` 才收集并写每变体每种子的轨迹 JSON，默认不累计详细轨迹。运行异常计失败，进程最终返回非零；有效命中后继续约 0.05 s，失败运行到统一时长或模型结束。重复使用输出目录会重写同名报告/轨迹。

窗口“保存结果”只写 `outputs/runs/viewer_<时间戳>.json`，内含当前状态及完整数值/飞控/视觉配置；曲线仅主动导出 CSV 到 `outputs/exports/`。当前入口不导出图片，完整文件契约见[输出规范](../../docs/输出规范.md)。

`--compare-v1` 对照使用原 v1 控制配置，但双方共用修补后的识别器，结果键为 `predictive` 与 `baseline_v1_same_vision`；它不是直接双击原 v1 窗口的逐字节复现。红十字在本分区仅显示，不参与外部预测导引。

历史 64 发验收属于旧运行记录，见[历史运行说明](../../docs/历史运行说明_2026-10-20.md)和已有输出。随机移动目标可能在飞镖到达前已停止，命中率不能直接表述为持续移动拦截能力。本次只核对源码和参数，不复核历史命中率、不证明实物能力。
