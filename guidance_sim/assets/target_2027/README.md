# 2027 活动靶模块运行资产

整理日期：2026-10-21。以 [manifest.json](manifest.json) 为正式数据源；当前只保留 `lite` 变体。由 [field_2027_lite](../field_2027_lite/README.md) 的 `target_assembly` 引用，`prototypes/field_scene.py` 负责加载和静态余量替换。

| 子目录 | 内容 |
|---|---|
| `assembly/lite/` | 1 个轻量外壳 STL |
| `assembly/shared/` | 传感器和灯部件的 2 个 STL，清单记录保留原始三角面 |
| `static/lite/` | 5 个已经移除活动模块的场地静态余量 STL |

清单记录源为原始 `RMUC2027_V1.0.0.STL`，未使用 STEP 几何或对齐；源 STL 的大小与 SHA256 已在本次整理中与 `local_originals/` 原件核对一致。源三角面编号区间、分离检查、静态排除部件和各网格哈希保留在清单中。

网格使用米、保留世界轴；随动模块与静态余量是成套数据。显示网格不增加碰撞和质量，接触与运动由模型和 profile 负责。本次核对 3 个随动网格、5 个静态余量的路径均存在，并核对清单所列 3 个随动网格 SHA256 一致；未重新进行几何分割验收。

清单中的 `provenance_file: provenance.json` 和 `generator: guidance_sim/scripts/split_2027_target.py` 当前都没有对应文件。因此目前可交接运行资源与清单记录，无法仅凭本目录完整复现原分割生成过程。原文件和清单本次未修改。
