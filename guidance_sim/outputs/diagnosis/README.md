# 飞控链路诊断结果交接

交接盘点：2026-10-21；原有47个文件、18,888,578字节。本次只读整理，报告属于2026-10-19阶段，未重新运行验证。返回[输出总索引](../README.md)。

## 作用与入口

本目录保存原物理飞控的单位/符号/执行量级诊断，以及独立有限响应理想导引后端的早期对照；它不是最终PN专家或传统控制增强版的主验收目录。

先读[implementation_plan.md](implementation_plan.md)和[current_model_report.json](current_model_report.json)。原记录明确：真实视觉N10案例未命中，几何误差约1.45247m；0.25g合成控制比力、0.10s响应、2.5g/s变化率是未标定工程假设。不能据此宣称当前最佳控制或实物已具备对应能力。

| 文件/子目录 | 类别与用途 |
|---|---|
| `units_sign_probe.py/.json`、`compensation_probe.py/.json` | 单位、符号、滚转光线补偿和抗饱和相关诊断探针；PY为当时的复查脚本 |
| `ballistic_response_audit.json`、`finite_response_independent_audit.json` | 有限响应、解析弹道/阻力、限幅、回放与积分收敛的历史数值核查 |
| `independent_yaw_actuation_probes.json` | 偏航执行链独立探针 |
| `baseline/`、`corrected/`及对应`.log` | 修改前后原物理模型对照；含当时配置、report、轨迹和视图截图 |
| `ideal_before_response/` | 引入有限响应前的旧简化结果，保留作过程对照 |
| `finite_response_vision/`、`finite_response_truth/`、`finite_response_yaw_plus1/` | 有限响应下真实视觉、显式真值及偏航+1°扰动对照；先读`result.json`，按需再读大型`trace.json` |
| `ballistic_cli_check/` | 独立CLI抛体/弹道校核结果与轨迹 |
| `pn_expert/` | PN专家前期像素中心、数学、集成和CLI独立审查；不是后续29场景最终汇总 |
| `dataset_alignment/`、`dataset_alignment_updated/`、`learning_audit/` | 学习数据接口的合成/小型边界样本；含`fake_pass/fake_fail`、`goal_pass/goal_fail`等，不属于正式训练集 |

## 对应源码与输入

主要对应 [prototypes/ideal_guidance_sim.py](../../prototypes/ideal_guidance_sim.py)、[guidedemo_control.py](../../prototypes/guidedemo_control.py)、[flight_sensors.py](../../prototypes/flight_sensors.py)、[dart_demo.py](../../prototypes/dart_demo.py) 和 [pn_expert_v2/expert.py](../../pn_expert_v2/expert.py)。输入以各结果旁保存的 `flight_config.json`、`vision_config.json` 及JSON内参数为准，不能把当前默认参数回填成历史参数。

后续阶段参见[PN专家验收](../pn_expert_acceptance/README.md)、[正式初始数据集](../expert_dataset/README.md)和[传统控制增强版](../traditional_limit/README.md)。不同模型的命中判定和执行能力假设不同，不能直接合并统计。

## 保留或重建

保留小型报告、探针、配置和前后对照；大轨迹用于逐步排错，交接备份应与对应`result/report`一起保留。合成NPZ也应保留原名，避免流入训练流水线。要重跑时核对原脚本导入的源码与模型版本，写入新的输出目录，不覆盖历史证据。本轮未运行目录内任何PY。
