# 运行资产交接

整理日期：2026-10-21。这里是桌面仿真直接加载的网格和清单，属于运行资源。原始 CAD 位于 [local_originals](../../local_originals/README.md)，运行时不依赖原始 CAD 文件。

## 分区与加载关系

| 分区 | 整理时原有内容 | 使用方 |
|---|---|---|
| [dart_shenda](dart_shenda/README.md) | `manifest.json` + 124 个 OBJ | `prototypes/simulation_setup.py` 选择默认镖体，`prototypes/dart_model.py` 读取外观、估计物性与相机数据 |
| [field_2027_lite](field_2027_lite/README.md) | `manifest.json` + 215 个 STL | `prototypes/field_scene.py` 的默认 2027 场地 |
| [target_2027](target_2027/README.md) | `manifest.json` + 8 个 STL | 由轻量场地的 `target_assembly` 引用，包含随动检测模块和静态余量替换 |
| [guide_light_2027](guide_light_2027/README.md) | `luminous_cap.stl` | `profiles/field_2027_fixed_target.json` 的 `guide_light.mesh_file`，由 `prototypes/dart_model.py` 读取 |

四区原有资源共 351 个文件。场地、镖体和活动模块的显示网格关闭碰撞与自动质量贡献；接触体、靶面判定和物理配置由模型代码及 profile 另行定义。资产外观不能代替实测气动、材料属性或官方裁判判定。

网格路径按各清单或发射 profile 的相对位置解析。复制运行环境时应一起保留四个子目录、清单和 `profiles/`，尤其不要漏掉场地引用的兄弟目录 `target_2027/`。

## 来源与当前缺项

- 镖体来源是参考装配 `三角翼制导镖.STEP`；场地及靶/灯资产记录来源为 `RMUC2027_V1.0.0.STL`。原件现位于 `local_originals/`，对应 STEP 与 STL 已在本次整理中只读核对 SHA256 一致。
- 镖体、场地清单和发射 profile 的部分 `source.path` / `field_source` 保留迁移前的根目录绝对路径，旧路径目前不存在。这是来源记录过时；实际运行网格使用相对路径。本次未修改清单。
- `target_2027/manifest.json` 引用的 `provenance.json`、`guidance_sim/scripts/split_2027_target.py` 不在当前工作区。
- 发射 profile 引用的 `guide_light_2027/provenance.json` 不在当前工作区。
- 轻量场地清单中历史 STEP 失败报告 `assets/field_2027/step_conversion_failures.json` 不在当前工作区，完整场地 `field_2027/` 也未保留。代码只会登记实际存在的场地，当前轻量版可被选择。

## 本次核对范围

本次检查清单、加载代码和文件存在性；未重建网格、重新转换 CAD、运行渲染或改变任何 JSON / OBJ / STL。运行路径完整性与历史溯源材料完整性应分别检查；上述缺失报告和生成器不应被当作已随交付保留。
