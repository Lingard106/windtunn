# geometry/ — 参数化弹体几何产物 (gen_airframe.m v1)

坐标系 B: x 前弹轴, y 右, z 下, 右手系, 原点=弹体几何中心。单位米。

| 文件 | 内容 | 消费方 |
|---|---|---|
| body.stl | 弹体旋转体网格 (nose=flat) | snappyHexMesh / MJCF asset |
| fin_{h_pos,h_neg,v_pos,v_neg}.stl | 4 舵面盒网格 (弹体系绝对坐标, 零舵偏) | snappyHexMesh |
| airframe_params.json | 几何/质量惯量/无量纲化基准/舵面命名与混控 | 查表换算 / MJCF inertial |
| glider.mjcf | MuJoCo 骨架 (freejoint+4 舵 hinge, 舵面 box 图元) | 视觉侧直接加载 |

- 无量纲化基准 (待 Phase 0 冻结): S_ref=0.01767 m² (弹身迎面积), L_ref=0.150 m (弹径), 力矩参考点=全系统质心。
- 舵面 STL 是零舵偏位形; MuJoCo 中舵面用 box 图元绕铰链转动, STL 仅供 CFD 装配与扫气标定。
- 改参数: gen_airframe(cfg) 覆盖字段 (见生成器头注释); MJCF 弹体 inertial 已按 profile 惯量写入 (cone 亦正确)。
