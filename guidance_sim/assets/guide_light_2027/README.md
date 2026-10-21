# 2027 灯面运行资产

整理日期：2026-10-21。本目录原有唯一文件为 `luminous_cap.stl`（109,084 字节），没有独立 `manifest.json`。运行数据和来源说明位于 `profiles/field_2027_fixed_target.json` 的 `guide_light` 字段，网格路径相对该 profile 解析。

profile 记录该网格从原始 `RMUC2027_V1.0.0.STL` 的穹顶透镜表面裁出，保留世界轴，已向外偏置 0.2 mm 以避免重叠闪烁。显示时不再重复偏置、旋转或缩放。亮度是渲染近似，未标定为实测光度模型。

`prototypes/dart_model.py` 负责读取灯面信息，`prototypes/dart_demo.py` 创建对应显示网格。此灯面与 [target_2027](../target_2027/README.md) 中的灯壳/模块资产承担不同显示职责，不能仅凭文件名互换。

profile 的 `guide_light.provenance.geometry_report` 指向本目录 `provenance.json`，该文件当前不存在。原始 STL 保留在 `local_originals/`；本次只核对网格和引用关系，没有恢复缺失报告或重新验证裁剪边界。
