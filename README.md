# WindTunnelSuite 工作区与交接入口

整理日期：2026-10-21。此目录包含仿真程序、固件参考示例、运行资源、历史实验结果和原始 CAD 资料。

**接手先读 [交接总览](docs/交接总览.md)，运行前读 [环境与运行](docs/环境与运行.md)。** 原 README 的完整内容已保存到 [历史运行说明（截至 2026-10-20）](docs/历史运行说明_2026-10-20.md)，其中的实验成绩与界面验收属于历史记录。

## 各部分文档

| 部分 | 交接文档 | 用途 |
|---|---|---|
| 仿真工程 | [guidance_sim](guidance_sim/README.md) | 运行入口、模块关系与数据流 |
| 共用仿真与界面 | [prototypes](guidance_sim/prototypes/README.md) | 原物理入口、简化入口、视觉、显示与诊断 |
| 独立传统增强分区 | [traditional_control](guidance_sim/traditional_control/README.md) | 独立入口和完整配置 |
| 四发连续演示 | [four_shot_demo](guidance_sim/four_shot_demo/README.md) | 复用原单发传统控制窗口，增加自动换发和一行计分，默认不写文件 |
| 简化模型专家分区 | [pn_expert_v2](guidance_sim/pn_expert_v2/README.md) | 专家代码与配置的边界 |
| 参考设置与数据采集 | [learning](guidance_sim/learning/README.md) | 采集器、数据格式与历史导出区别 |
| 配置 | [profiles](guidance_sim/profiles/README.md) | 场地、物性、视觉和入口对应关系 |
| 运行资产 | [assets](guidance_sim/assets/README.md) | 必需网格、来源和清单 |
| 来源元数据 | [reference](guidance_sim/reference/README.md) | 原件哈希与来源记录 |
| 仿真输出与历史资料 | [outputs](guidance_sim/outputs/README.md) | 显式保存位置、数据采集与六组历史基线 |
| 固件参考示例 | [firmware_examples](firmware_examples/README.md) | 示例范围、平台依赖与未接入部分 |
| STM32 实际工程 | [Guidedemo](Guidedemo/README.md) | 后续加入的工程结构、源码与历史构建产物说明 |
| 原始 CAD 资料 | [local_originals](local_originals/README.md) | 原件保管与资源关系 |

现有专题说明保留在原位置：[控制策略与 OpenMV／STM32 移植说明](控制策略与OpenMV_STM32移植说明.md)。其原有技术内容未在本次整理中改写。

## 运行入口

本次整理的六个既有入口中，根目录保留两个常用入口，其余四个对照或实验入口集中在[启动器目录](启动器/README.md)。六个入口均使用 `guidance_sim/.venv/Scripts/python.exe`，支持转发命令行参数。旧名称已移除，没有保留重复别名。

| 启动器 | 原有用途 |
|---|---|
| [启动器/运行基线物理仿真.cmd](启动器/运行基线物理仿真.cmd) | 原完整物理仿真入口 |
| [运行传统控制仿真.cmd](运行传统控制仿真.cmd) | 独立传统增强分区入口 |
| [启动器/运行标准导引仿真.cmd](启动器/运行标准导引仿真.cmd) | 简化运动模型对照入口 |
| [启动器/运行专家策略仿真.cmd](启动器/运行专家策略仿真.cmd) | 简化模型专家入口 |
| [启动器/运行参考点微调仿真.cmd](启动器/运行参考点微调仿真.cmd) | 带参考设置的简化入口 |
| [采集专家示范数据.cmd](采集专家示范数据.cmd) | 批量数据采集入口，会新增输出 |

四发演示使用[启动四发赛场演示.cmd](启动四发赛场演示.cmd)，直接复用原单发传统控制窗口，仅增加连续换发与一行计分。代码只保留`four_shot_demo/run.py`，默认不写文件；此前的回放窗口和临时产物已撤掉。

上述入口均为仿真方案。`firmware_examples` 仍是参考示例，后续加入的 `Guidedemo` 是实际 STM32 工程，本次未修改其控制代码。当前未见训练器或神经网络模型交付。

## 交接配套资料

- [环境与运行](docs/环境与运行.md)：本机已核对版本、环境重建方法、启动注意点。
- [输出规范](docs/输出规范.md)：默认不落盘、显式结果/CSV/轨迹开关与采样文件契约。
- [工作区文件清单](docs/工作区文件清单.md)：整理前文件数、体积与逐文件清单。
- [整理记录](docs/整理记录.md)：改动范围、保留规则和检查结果。
- [交接清单 JSON](docs/交接清单_2026-10-21.json)：原文件清单与 SHA-256 基线。

当前根目录没有 `.git`；`.gitignore` 和 `.gitattributes` 只是保留的规则文件。交接时需单独确认包含 `local_originals/` 和 `guidance_sim/outputs/` 的资料副本，不能据忽略规则推断它们已经备份。
