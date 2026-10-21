# PN专家v2验收结果交接

交接盘点：2026-10-21；原有175个文件、12,869,445字节。证据来自2026-10-19阶段；本轮没有重跑任何场景。返回[输出总索引](../README.md)。

## 最终应读的记录

先读[implementation_plan.md](implementation_plan.md)，再读[acceptance_report.json](acceptance_report.json)和[acceptance.png](acceptance.png)。汇总记录29个固定场景专家命中29/29、关闭控制3/29，`all_pass=true`；其范围是原30mm半径固定靶的首次镖头过面几何判定。使用理想自身状态、声明的静态灯平面/偏移先验及假设有限执行响应，不是实飞或官方裁判验收，也没有NN训练。

29场景构成为标称1、组合边界8、随机留出20。原冻结扰动包括XYZ各±0.05m、侧风±1m/s、偏航±0.2°、俯仰±0.1°、速度±0.05m/s；实际场景清单以[frozen_scenarios.json](frozen_scenarios.json)为准。

## 原始结果与最终汇总的关系

| 子目录/文件 | 阶段及用途 |
|---|---|
| `nominal_development/` | 开发标称单例的`result.json`和轨迹，用于早期排错 |
| `nominal_v1/` | 单标称冻结验收；保存专家/关闭控制报告、视觉及轨迹 |
| `frozen_batch_v1/` | 原29场景批次，按`nominal/corner/heldout`保存专家与关闭控制对照；原`report.json`为27/29、`all_pass=false`，原样留存 |
| `substep_refinement/` | `heldout_06`与`heldout_17`两例的控制子步交点复核，历史报告2/2通过 |
| `acceptance_report.json` | 合并原批次与两例复核后的交付汇总，含`refinement`解释、前后源码哈希、约束及逐场景结果 |
| `traditional_smoke/` | 原物理入口兼容冒烟，不纳入专家29场景统计 |
| `acceptance_probe.py` | 当时的冻结场景验证脚本，不是产品控制器 |

原批次两例涉及宏步与物理控制tick子步端点插值导致的几何相等检查差异；汇总说明重查交点后通过、产品哈希不变。差异约为0.0000003335m和0.0000028098m，不能把原失败标志删掉后伪称从未失败，也不能忽略后续复核仅引用27/29。

## 对应代码、输入与保留建议

产品算法：[pn_expert_v2/expert.py](../../pn_expert_v2/expert.py)及[config.json](../../pn_expert_v2/config.json)。仿真接入：[prototypes/ideal_guidance_sim.py](../../prototypes/ideal_guidance_sim.py)；原视觉源由`flight_sensors.py`、`openmv_tracker.py`等提供。验收输入以冻结场景、汇总保存的`vision_config`、`expert_config`、`dynamics`、`numeric_baseline`和源码哈希为准。

保留最终汇总、原冻结批次、两例复核和所有NPZ轨迹/视觉报告的对应关系；`*_expert_trace.npz`及`*_off_trace.npz`可用于逐步复算，不能只保留图片。需要重跑时先检查`acceptance_probe.py`参数与当前源码兼容性，使用新输出目录；本次没有确认旧脚本对当前修改后源码仍可完全复现。后续正式专家数据位于[expert_dataset](../expert_dataset/README.md)，真实刚体控制增强版位于[traditional_limit](../traditional_limit/README.md)，统计口径不同。
