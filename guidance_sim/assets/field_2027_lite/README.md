# 2027 轻量场地运行资产

整理日期：2026-10-21。以 [manifest.json](manifest.json) 为正式数据源，含 215 个 STL 分块；`prototypes/field_scene.py` 在存在时优先选择本目录。

清单记录来源为 `RMUC2027_V1.0.0.STL`，原文件现位于 `local_originals/`，旧 `source.path` 仍指工作区根目录。源 STL 为毫米，转换后网格使用米、世界 Z 向上，已应用清单中的缩放和平移。轻量转换记录为 12 mm 栅格聚类、386,229 个三角面；这些是已有清单记录，本次未重新转换。

`target_assembly` 指向 [target_2027](../target_2027/README.md) 的 `lite` 变体。加载器会替换 5 个场地静态余量分块，并把 3 个靶模块网格交给随动靶体显示。交接时需要同时保留该兄弟目录；网格和静态余量不能随意混用。

场地仅提供视觉外观，所有场地网格碰撞关闭。清单中的原点、整体简化和历史摆放记录不等于完整官方坐标标定；当前发射与靶面信息另见 `profiles/field_2027_fixed_target.json`。

历史 `conversion.step_fallback.report` 指向 `assets/field_2027/step_conversion_failures.json`，该报告及完整场地目录当前缺失；现有清单说明部分 STEP 定义三角化失败后改用 STL。当前工作区没有恢复这段转换流程所需的完整历史工具/报告。本次已核对本清单的 215 个网格引用存在，未做渲染或几何误差复验。
