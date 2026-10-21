# 专家数据与学习接口交接

交接盘点：2026-10-21；原有104个文件、12,742,423字节。本次未采集、训练或重新审计数据；下述完成状态来自保存的manifest/report。返回[输出总索引](../README.md)。

## 应使用哪一批

| 子目录 | 状态 | 用途与阅读顺序 |
|---|---|---|
| [initial_sl_v1](initial_sl_v1/README.md) | **正式冻结的初始数据集**；历史`collection_status=complete` | 先读原README，再读`report.json`、`independent_audit.json`与`sl_ready/metadata.json`；包含完整原始回合及可信行导出 |
| `pilot_v3/` | 完成的3回合先导批次 | train/validation/test各1回合；用于v3参考接口和采集链路冒烟，不代替72回合正式集 |
| `20261020T011251Z_210c31/` | **未完成批次**；manifest为`in_progress` | 实际保留5个train NPZ及配置/manifest，没有完成汇总；不与正式集自动合并，也不当验证/测试集 |
| `reference_checks/` | 参考接口历史检查 | `report.json`保存参考方向、模式等检查结果，属于功能证据 |
| `legacy_v2_smoke/` | 旧v2兼容冒烟 | `result.json`与大型`trace.json`；不属于监督训练数据 |

根[implementation_plan.md](implementation_plan.md)记录当时“只接目标参考与采专家数据，不训练NN”的实施边界。正式集72回合按整回合分为48训练、12验证、12测试；原始6959行，可信标签4247行，原靶命中71/72。唯一失败`train_00037`保留在原始集，其96行可信掩码全零。

## 数据接口必须保留的含义

`initial_sl_v1/sl_ready/`为可信行导出：train 2806、validation 720、test 721行；31维动作前特征，2维标签`[pitch, yaw]`，单位g，向下/向右为正，表示排除重力的专家控制比力请求。标签不是舵角，也不是已实现的实际过载。特征/单位/哈希以[metadata.json](initial_sl_v1/sl_ready/metadata.json)为准。

原始 `train/`、`validation/`、`test/` 每回合一个NPZ，包含有效/可信掩码与`eval_*`评价真值。学习输入不能包含`eval_*`，不能把失败回合中的零指令当作可信标签。整回合分区已冻结；不要按行重新随机划分。现有数据未标准化，后续归一化统计只在train拟合。

可信导出主要含目标有效时的专家决策。失锁和过期目标的显式门控仍需在未来控制器中保留，不能以为当前监督样本已经教授这些处置。历史独立审计记录了7处相邻保存行的上一条指令不等于前行标签，这是20ms专家决策之间的目标超时撤销指令；原审计解释为真实动作前状态，无未来状态泄漏。

## 来源与重建

采集入口：[learning/dataset.py](../../learning/dataset.py)及根[采集专家示范数据.cmd](../../../采集专家示范数据.cmd)。目标参考实现：[learning/aim_reference.py](../../learning/aim_reference.py)；专家：[pn_expert_v2/expert.py](../../pn_expert_v2/expert.py)；渲染/简化后端：[prototypes/ideal_guidance_sim.py](../../prototypes/ideal_guidance_sim.py)。每批旁的`collection_config.json`、`aim_reference_config.json`、`vision_config.json`和`manifest.json`冻结了输入、种子计划、版本、源码哈希和工况。

本集使用真实渲染视觉、理想自身状态、静态CAD先验和假设执行能力（0.25g、0.10s、2.5g/s），只覆盖固定靶局部扰动。旧审计中的“源码哈希匹配”描述当时审计，当前代码后续已有开发，不能直接视为今天重新审计通过。

完整保留正式集的原始NPZ、manifest、独立审计、配置和导出，不只复制`sl_ready/train.npz`。未完成批次保留供溯源；要续采先检查工具是否支持可靠恢复并核对哈希，否则重新采集到新目录。当前没有NN模型、训练检查点或训练验收产物；本轮也未生成这些内容。
