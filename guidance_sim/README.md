# guidance_sim 仿真运行区交接

本目录保存可运行的 Python 仿真、参数、运行资产及本机结果。交接盘点日期为 2026-10-21；本次仅核对源码、JSON 与启动器，未启动窗口、仿真、批量采样或训练。工作区总导航见[根 README](../README.md)，历史结论与旧操作说明见[历史运行说明](../docs/历史运行说明_2026-10-20.md)。

## 目录职责

| 路径 | 职责与接手入口 |
|---|---|
| [prototypes](prototypes/README.md) | 共同的 MuJoCo 运行底座、原 v1 飞控、视觉、窗口、回放，以及理想导引入口 |
| [traditional_control](traditional_control/README.md) | 保留完整刚体/气动/执行机构的传统控制增强入口 |
| [pn_expert_v2](pn_expert_v2/README.md) | 纯数值弹道预测专家与独立参数；通过理想导引入口运行 |
| [learning](learning/README.md) | 红十字参考转换与专家数据采样，当前没有神经网络训练程序 |
| [four_shot_demo](four_shot_demo/README.md) | 原单发传统控制窗口的四发实时排程，仅一个run.py，无自动输出 |
| [profiles](profiles/README.md) | 场地发射预设、数值物性、原 v1、理想导引与桌面视觉参数 |
| [assets](assets/) | 运行所需网格和 manifest，文件依赖应连同相对路径保留 |
| [reference](reference/) | 原飞控来源元信息；不是完整原固件工程 |
| [outputs](outputs/README.md) | 显式保存的结果与数据、既有历史证据；交接时另行保管 |
| `.venv` | 本机 Python 运行环境，不能作为可迁移源码包 |
| [requirements.txt](requirements.txt) | 四项锁定的第三方运行依赖 |

## 运行入口

以下命令均在工作区根目录 `D:\WindTunnelSuite` 的 PowerShell 执行。本次整理的六个既有入口都显式使用 `guidance_sim/.venv/Scripts/python.exe`，并将附加参数传给 Python；其中常用仿真和数据采集保留在根目录，其余放在 `启动器/`。另有并行新增四发演示入口，见表后说明。

| 启动器 | 实际入口与关键选择 |
|---|---|
| [运行基线物理仿真.cmd](../启动器/运行基线物理仿真.cmd) | `prototypes/dart_viewer.py --season 2027 --flight-control on --flight-config guidance_sim/profiles/flight_control_v1_direct_2027.json --auto-run` |
| [运行传统控制仿真.cmd](../运行传统控制仿真.cmd) | `traditional_control/run.py --auto-run`；默认随机移动靶 |
| [运行标准导引仿真.cmd](../启动器/运行标准导引仿真.cmd) | `prototypes/ideal_guidance_sim.py --guidance-source vision --auto-run`；默认 `profiles/ideal_guidance.json` |
| [运行专家策略仿真.cmd](../启动器/运行专家策略仿真.cmd) | 理想导引入口加载 `pn_expert_v2/config.json`，默认不写文件 |
| [运行参考点微调仿真.cmd](../启动器/运行参考点微调仿真.cmd) | v2 配置加 `learning/aim_reference.json`，默认不写文件 |
| [采集专家示范数据.cmd](../采集专家示范数据.cmd) | `learning/dataset.py`；默认 48/12/12 个训练/验证/测试回合 |

新增的[启动四发赛场演示.cmd](../启动四发赛场演示.cmd)直接复用单发时期的传统控制窗口，保留原渲染、镜头、曲线、调参及独立相机，仅增加四发实时排程和计分。已撤掉另做的回放界面、展示场景及后台预计算；默认不写文件。用法见[四发演示说明](four_shot_demo/README.md)。

进入子模块 README 可查看输入和参数边界，统一文件契约见[输出规范](../docs/输出规范.md)。普通窗口和无窗口运行默认不写文件；无窗口结果打印到控制台。窗口“保存结果”只写 `outputs/runs/viewer_<时间戳>.json`，曲线仅在主动导出时写 `outputs/exports/` 下 CSV，不导出图片。传统控制和理想导引仅在 `--headless` 且显式指定 `--output` 时写 `result.json`，详细轨迹另需 `--trace`。

## 环境与依赖

当前依赖锁定为 MuJoCo 3.16.0、NumPy 1.26.4、opencv-python 4.10.0.84、Pillow 12.3.0。窗口使用标准库 Tkinter；`--headless` 表示不打开主窗口，真实图像路线仍需 MuJoCo 渲染能力。

本机交接检查确认 `.venv` 为 Python 3.11.9、Tk 8.6，四项版本与依赖文件一致，`pip check` 通过。系统默认 Python 为 `C:\Python314\python.exe`，运行时应明确使用本目录虚拟环境。环境重建方法和本次验证范围见[环境与运行](../docs/环境与运行.md)。

```powershell
.\guidance_sim\.venv\Scripts\python.exe -m pip check
.\guidance_sim\.venv\Scripts\python.exe -B -X utf8 guidance_sim/learning/dataset.py --dry-run
```

第二条仅展示现有采样计划，不执行图像渲染或采样；本次整理未实际执行该命令。

## 交接时必须区分的状态

- 原 v1、完整物理增强版、理想传统 PN、理想 v2 是不同入口。`flight_control_default.json` 也不等于基线物理仿真启动器显式加载的配置；比较结果应保存入口、完整生效配置及输入哈希。
- 场地网格只用于外观；刚体接触与几何穿面有各自的判定。接触命中距离、镖尖过面误差和红十字所选目标误差不能互换。
- 气动、舵效、理想过载能力及带宽仍是工程假设；源码和历史结果不代表实物标定或官方裁判判定。
- 视觉使用真实渲染图，但部分路径使用理想自身位姿和静态场地标定。显式 `truth` 路线属于真值对照，需在结果中保留标记。
- 当前 `profiles/vision_desktop.json` 的红十字为 `[160,125]`，v3 参考标定零点为 `[160,180]`。v3 的 `image_reference` 模式会将差值转换为靶面偏移；接手前先核对当前实验意图，不能照搬历史零偏移说明。
- 现有仓区未包含自动化测试目录或完整训练流水线。本次只确认文件与调用关系，历史命中率与验收结果未重新运行验证。

本次文档根据本次整理的六个既有入口、各入口的 `main()`、配置加载函数和现有 JSON 编写，另补充并行新增四发演示入口的静态说明；源码、控制参数、资源和已有结果均未因本次文档整理而修改。
