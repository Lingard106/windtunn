# 仿真输出与历史证据交接

交接盘点日期：2026-10-21。本目录保存既有诊断、数据采集、控制验收和界面验收结果。本次只读盘点并补交接文档，没有重新运行仿真、采集、训练或验收；下述数值均为原报告的历史记录。

## 当前输出契约

当前源码以[输出规范](../../docs/输出规范.md)为准。普通窗口与无窗口仿真默认不写文件；当前入口不导出图片。窗口“保存结果”写 `runs/viewer_<时间戳>.json`，曲线主动导出 CSV 写 `exports/`。传统控制、简化导引和底层入口仅显式指定输出目录时写 `result.json`；详细轨迹还需 `--trace`，传统控制和简化导引同时要求无窗口模式。四发无窗口入口显式指定的是单个 JSON 文件路径。

新专家采集只写每回合 NPZ 和包含配置/最终 `summary` 的 `manifest.json`，不另写重复配置或 `report.json`。下方历史数据保留原有文件与数据契约。

## 原盘点的历史结果集

统计取自新增本轮交接文档前的文件系统；包含隐藏文件、缓存和子目录全部文件，字节为逻辑长度，不是磁盘占用。

| 目录 | 内容与阅读入口 | 原有文件数 | 原有字节数 | 约 MiB |
|---|---|---:|---:|---:|
| [diagnosis](diagnosis/README.md) | 原飞控量级/符号诊断、有限响应简化后端、PN专家前期检查 | 47 | 18,888,578 | 18.01 |
| [expert_dataset](expert_dataset/README.md) | 初始专家数据、可信标签导出、先导和未完成批次 | 104 | 12,742,423 | 12.15 |
| [pn_expert_acceptance](pn_expert_acceptance/README.md) | PN专家v2冻结场景验收、原始结果与子步复核 | 175 | 12,869,445 | 12.27 |
| [traditional_limit](traditional_limit/README.md) | 真实刚体传统控制增强方案及公差留出验收 | 87 | 16,553,444 | 15.79 |
| [v1_debug_design](v1_debug_design/README.md) | 三轴控制/调试显示设计、改动前快照、GUI与回放证据 | 135 | 13,876,359 | 13.23 |
| [v1_ui_refinement](v1_ui_refinement/README.md) | 后续UI响应性能、紧凑调参和波形旁编辑验收 | 21 | 438,780 | 0.42 |
| **合计** | **原盘点6个一级结果目录** | **569** | **75,369,029** | **71.88** |

本次四发演示曾生成的`outputs/four_shot_demo/`临时检查、图片与重复回放已按用户要求删除。[当前四发入口](../four_shot_demo/README.md)改为复用单发窗口实时运行，默认不再生成输出。上表原有历史数据不受此次清理影响。

原盘点时不存在 `outputs/pn_expert_v2/`、`outputs/reference_v3/`、`outputs/flight_debug/`。专家策略与参考点微调启动器现已取消默认输出目录，曲线 CSV 改为显式保存到 `outputs/exports/`；历史文字中的旧路径不能视为当前自动输出承诺。源码目录 `guidance_sim/pn_expert_v2/` 则确实存在。

## 接手时先确认

- 传统控制增强版先读 [traditional_limit/验收说明.md](traditional_limit/验收说明.md)，再读 `final_audit.json` 和小公差 `tolerance_holdout*`；不要将开发批次、旧名义工况或异常中断批次当作主验收。
- 准备监督学习先读 [initial_sl_v1/README.md](expert_dataset/initial_sl_v1/README.md)，使用其 `sl_ready/` 和元数据。`20261020T011251Z_210c31/manifest.json` 仍标记 `in_progress`，不能当作完整训练集。现有记录没有训练NN。
- PN专家验收先读 `pn_expert_acceptance/acceptance_report.json`；它组合原冻结批次与两例子步交点复核。原冻结报告中的失败标志被原样保留，不能单看其中一个报告下结论。
- 查看界面现状应结合 `v1_ui_refinement/` 的后续改进。`v1_debug_design/source_before/` 是历史源码快照，`preview.png` 是早期设计，不应覆盖当前源码或当作最新界面截图。

## 保留与复现规则

根 [.gitignore](../../.gitignore) 忽略整个 `guidance_sim/outputs/`，普通Git克隆不会带回这里的报告、交接README和数据。交接时须另外打包整个目录，或至少保留每个结果集的报告、配置、manifest/哈希、种子、原始轨迹/NPZ及其说明。核心入口和环境参见[项目交接总览](../../docs/交接总览.md)。

本轮未移动或删除任何文件。JSON/NPZ/CSV是结果证据，目录内PY是当时的探针/报告脚本，`source_before/`、`prediction_model_before/` 是历史快照；实际运行源码仍位于 `guidance_sim/prototypes/`、`pn_expert_v2/`、`learning/`、`traditional_control/`。

需要重建时使用原配置、种子、运行时版本与源码哈希核对依赖，输出到新目录并记录新时间。旧manifest里的“源码未变化”“审计通过”仅描述原生成时刻，本次未对其执行新一轮数值验证。`__pycache__`、`.pyc`、`.o`、探针`.exe`可由相应工具链重建，但本轮也完整保留；不要为了腾空间先删掉尚未单独备份的冻结证据。
