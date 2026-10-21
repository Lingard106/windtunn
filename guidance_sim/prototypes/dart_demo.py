"""MuJoCo 镖体仿真：默认开环，可选 Guidedemo 飞控与 IMU 噪声。

镖体坐标：X 向前、Y 向左、Z 向上；世界 Z 向上；四元数顺序为 w,x,y,z。
视觉从真实渲染图供给；气动/执行机构仍为估计，尚无 CFD 或官方裁判判定。
"""

import argparse
import copy
import json
import math
from collections.abc import Mapping
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

if __package__:
    from .aerodynamics import (AeroState, evaluate_aerodynamics,
                               load_default_aero_parameters, resolve_aero_model)
    from .dart_model import load_dart_model, load_launch_profile
    from .field_scene import load_field_scene, resolve_scene_manifest
    from .target_motion import TargetMotion, TARGET_MODES
    from .guidedemo_control import FlightController, load_flight_config
    from .control_actuation import ServoActuation, estimated_control_loads
    from .flight_sensors import imu_from_state, vision_from_detection, rotation_from_imu
    from .imu_noise import ImuNoise
else:
    from aerodynamics import (AeroState, evaluate_aerodynamics,
                              load_default_aero_parameters, resolve_aero_model)
    from dart_model import load_dart_model, load_launch_profile
    from field_scene import load_field_scene, resolve_scene_manifest
    from target_motion import TargetMotion, TARGET_MODES
    from guidedemo_control import FlightController, load_flight_config
    from control_actuation import ServoActuation, estimated_control_loads
    from flight_sensors import imu_from_state, vision_from_detection, rotation_from_imu
    from imu_noise import ImuNoise


CONFIG_PATH = Path(__file__).with_name("dart_demo_config.json")


def load_config(path=None):
    """读取默认数字参数，再用用户配置覆盖；场地路径单独处理。"""
    config = load_default_aero_parameters()
    config.update(json.loads(CONFIG_PATH.read_text(encoding="utf-8")))
    if path is not None and Path(path).resolve() != CONFIG_PATH.resolve():
        config.update(json.loads(Path(path).read_text(encoding="utf-8")))
    return config


def validate_config(config):
    """在建立模型前检查演示参数，输入错误时保留旧仿真。"""
    for name, value in config.items():
        if not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError("Config values must be finite numbers: %s" % name)
    for name in ("mass_kg", "rho_kg_m3", "reference_area_m2", "reference_length_m",
                 "dt_s", "t_max_s", "launch_height_m"):
        if config[name] <= 0:
            raise ValueError("%s must be positive" % name)
    if config["dt_s"] > .01:
        raise ValueError("Demo dt_s must be <= 0.01")
    for name in ("drag_coefficient", "normal_coefficient", "stability_coefficient",
                 "angular_damping_coefficient", "speed_mps"):
        if config[name] < 0:
            raise ValueError("%s must be nonnegative" % name)
    if not 5 <= config["camera_fov_deg"] <= 150:
        raise ValueError("camera_fov_deg must be between 5 and 150")
    if not 0 <= config.get("guide_light_brightness", .8) <= 1:
        raise ValueError("guide_light_brightness must be between 0 and 1")
    return dict(config)


def passive_aero(velocity_W, rotation_WL, omega_W, config):
    """保留旧气动入口；默认估计后端返回世界系重心载荷 N / N·m。"""
    state = AeroState(0., np.asarray(velocity_W), np.asarray(rotation_WL),
                      np.asarray(omega_W), np.array([0., config["wind_cross_mps"], 0.]))
    loads = resolve_aero_model().evaluate(state, config)
    return loads.force_W_N, loads.torque_W_Nm


def _fin_assets():
    """用小型三角网格画固定尾翼；它们暂时没有舵机或活动关节。"""
    meshes, geoms = [], []
    for axis in (1, 2):
        for sign in (-1, 1):
            name = "fin_%d_%d" % (axis, sign)
            vertices = []
            for thickness in (-.0015, .0015):
                for x, span in ((-.16, .015), (-.045, .015), (-.16, .105)):
                    vertex = [x, 0., 0.]
                    vertex[axis] = sign * span
                    vertex[3 - axis] = thickness
                    vertices.extend(vertex)
            meshes.append('<mesh name="%s" vertex="%s"/>' %
                          (name, " ".join(str(x) for x in vertices)))
            geoms.append('<geom type="mesh" mesh="%s" rgba="0.12 0.55 0.72 1" '
                         'contype="0" conaffinity="0" group="1"/>' % name)
    return "\n".join(meshes), "\n".join(geoms)


def _target_collision_xml(profile, dart_model):
    """生成独立靶板与镖头接触体，保持 CAD 外观和接触简化相互独立。

    靶面局部 Z 指向背侧，薄盒前表面恰好落在原几何判定平面；不将
    盒子中心误放在靶面上，以免碰撞发生在目标前方半个厚度的位置。
    短圆柱前表面与 nose_tip 的局部 X 一致，半径/厚度为参考 CAD 触发头近似。
    """
    if profile is None or not profile.target_collision.enabled:
        return "", ""
    contact = profile.target_collision
    normal = np.asarray(profile.target_normal_W)
    horizontal = np.cross([0., 0., 1.], normal)
    if np.linalg.norm(horizontal) < 1e-8:
        horizontal = np.array([1., 0., 0.])
    horizontal /= np.linalg.norm(horizontal)
    vertical = np.cross(normal, horizontal)
    center = np.asarray(profile.target_position_W_m) + normal * contact.thickness_m/2
    values = lambda array: " ".join(str(float(value)) for value in array)
    target = f'''<geom name="target_collision" type="box" pos="{values(center)}"
        xyaxes="{values(np.r_[horizontal, vertical])}"
        size="{values([*contact.half_extents_m, contact.thickness_m/2])}"
        rgba=".15 .18 .2 0" contype="2" conaffinity="2" priority="1" condim="3"
        friction="{values(contact.friction)}" solref="{values(contact.solref)}"
        solimp="{values(contact.solimp)}" margin="0" gap="0"/>'''
    nose = np.array([.215, 0., 0.] if dart_model is None else dart_model.nose_tip)
    # MuJoCo 圆柱轴默认是局部 Z；绕 Y 转 90° 后轴沿镖体 +X。
    # 短圆柱中心后退半个厚度，前圆面中心就是清单中的 nose_tip。
    cylinder_center = nose - np.array([contact.nose_length_m/2, 0., 0.])
    tip = f'''<geom name="nose_contact" type="cylinder" pos="{values(cylinder_center)}"
        quat=".7071067811865476 0 .7071067811865476 0"
        size="{contact.nose_radius_m} {contact.nose_length_m/2}" rgba="1 .5 .1 0" density="0"
        contype="3" conaffinity="3" margin="0" gap="0"/>'''
    return target, tip


def _guide_light_metadata(profile, brightness):
    """统一绘制和保存所用的灯几何；旧方案仍采用原演示球体和原偏移。"""
    if profile.guide_light is not None:
        result = profile.guide_light.metadata()
    else:
        normal = np.asarray(profile.target_normal_W)
        horizontal = np.cross([0., 0., 1.], normal)
        if np.linalg.norm(horizontal) < 1e-8:
            horizontal = np.array([1., 0., 0.])
        horizontal /= np.linalg.norm(horizontal)
        vertical = np.cross(normal, horizontal)
        vertical /= np.linalg.norm(vertical)
        if vertical[2] < 0:
            vertical = -vertical
        center = (np.asarray(profile.target_position_W_m) - normal*.012
                  + horizontal*.02 - vertical*.12)
        result = {"shape": "legacy_sphere", "center_W_m": center.tolist(),
                  "radius_m": .0275, "diameter_m": .055,
                  "provenance": {"method": "未配置原始透镜网格，保留旧演示球体与靶面偏移"}}
    result["brightness"] = max(0., min(1., float(brightness)))
    return result


def build_xml(config, field_scene=None, dart_model=None, launch_position=None, *,
              suppress_demo_props=False, launch_profile=None, guide_light_brightness=0.8,
              target_mode="fixed"):
    """生成 MuJoCo 的 MJCF/XML 世界说明，描述物体、重力、相机和显示资源。

    asset 声明可复用网格/材质，worldbody 将它们放到世界；body 是物体，
    freejoint 允许镖体自由移动转动，inertial 填质量和转动惯量。
    场地网格只负责外观；镖体清单另外提供估计物性。launch_position 是
    米制世界 XYZ，表示镖体重心离架瞬间的位置，不模拟发射架内加速。
    suppress_demo_props 用于没有可视场地的数值求解，移除旧演示障碍物，
    保留与 CAD 场景一致的地面碰撞，确保省略外观不会改变动力学。
    """
    meshes, fins = _fin_assets()
    collision_enabled = launch_profile is not None and launch_profile.target_collision.enabled
    target_geom, nose_geom = _target_collision_xml(launch_profile, dart_model)
    # 高速小头撞薄板必须使用细步长。窗口读取 model 的有效 dt，而原配置
    # 保留用户请求值；重放/标定也应使用同一有效步长，元数据会明确记录。
    # MJCF 采用界面配置的宏步长。真正接近靶板/地面时，step() 会临时
    # 将 MuJoCo 步长细分；远离接触面的长距离飞行不必一直承担最小步长。
    effective_dt = config["dt_s"]
    position = launch_position if launch_position is not None else [1., 0., config["launch_height_m"]]
    position_text = " ".join(str(value) for value in position)
    if dart_model is None:
        # 未加载 CAD 时使用兜底演示惯量，随质量等比例变化；CAD 分支另用
        # 清单中的几何/质量分布估计惯量，两者都还不是实测标定值。
        scale = config["mass_kg"] / .15
        dart_meshes = meshes
        inertia = f'diaginertia="{.00004*scale} {.002*scale} {.002*scale}"'
        dart_geoms = f'''
      <geom name="hull" type="capsule" fromto="-.16 0 0 .12 0 0" size=".017"
            rgba=".82 .85 .88 1" friction=".4 .02 .001"/>
      <geom type="ellipsoid" pos=".155 0 0" size=".06 .017 .017"
            rgba=".93 .47 .18 1" contype="0" conaffinity="0"/>
      {fins}'''
        if collision_enabled:
            # 旧演示的镖身胶囊本来就明显短于椭球头，允许随后也与靶板接触。
            dart_geoms = dart_geoms.replace('friction=".4 .02 .001"',
                'friction=".4 .02 .001" contype="3" conaffinity="3"')
        camera_position, camera_axes = ".23 0 0", "0 -1 0 0 0 1"
    else:
        dart_meshes = dart_model.xml_assets()
        # UI 改质量时，假定内部质量分布形状不变，完整惯量同比例缩放。
        mass_scale = config["mass_kg"] / dart_model.mass_kg
        values = " ".join(str(value * mass_scale) for value in dart_model.full_inertia)
        inertia = f'fullinertia="{values}"'
        # 胶囊只近似镖身接地碰撞，半径取 17 mm；真实翼面网格不参与碰撞。
        # fromto 是胶囊中轴的两个端点，因此各缩短一个半径以靠近外形总长。
        radius = min(.017, (dart_model.bounds_max[0] - dart_model.bounds_min[0]) / 4)
        rear = dart_model.bounds_min[0] + radius
        front = dart_model.bounds_max[0] - radius
        hull_axis_yz = (0., 0.)
        if collision_enabled:
            # 旧胶囊包到头部最前端，倾斜靶面可能先碰到其圆帽而不是镖头。
            # 将镖身前端后缩，独立短圆柱负责头部接触。它仍只是细镖身近似，
            # 不模拟翼面/零件碰撞；大角度侧撞可以合法地先碰到镖身。
            front = max(rear + .001, min(front, dart_model.nose_tip[0] - .030))
            hull_axis_yz = dart_model.nose_tip[1:]
        hull_mask = 'contype="3" conaffinity="3"' if collision_enabled else ''
        axis_y, axis_z = hull_axis_yz
        dart_geoms = f'''
      <geom name="hull" type="capsule" fromto="{rear} {axis_y} {axis_z} {front} {axis_y} {axis_z}" size="{radius}"
            rgba=".7 .7 .7 0" friction=".4 .02 .001" {hull_mask}/>
      {dart_model.xml_geoms()}'''
        camera_position = " ".join(str(value) for value in dart_model.camera_position)
        camera_axes = " ".join(str(value) for value in dart_model.camera_xyaxes)
    if field_scene is None and suppress_demo_props:
        scene_meshes, scene_geoms = "", ""
        floor_appearance = 'material="floor"'
    elif field_scene is None:
        scene_meshes = ""
        scene_geoms = '''
    <geom type="box" pos="14 -7.5 .025" size="14 .025 .025" rgba="0.9 0.9 0.9 1"/>
    <geom type="box" pos="14 7.5 .025" size="14 .025 .025" rgba="0.9 0.9 0.9 1"/>
    <geom type="box" pos="0 0 .025" size=".025 7.5 .025" rgba="0.9 0.9 0.9 1"/>
    <geom type="box" pos="28 0 .025" size=".025 7.5 .025" rgba="0.9 0.9 0.9 1"/>
    <geom type="box" pos="1 0 .25" size=".55 .4 .25" rgba="0.55 0.59 0.62 1"/>
    <geom type="box" pos="25 -1.8 1.4" size=".04 .4 .65" rgba=".12 .13 .15 1"/>
    <geom type="sphere" pos="24.95 -1.8 1.4" size=".06" material="beacon"/>
    <geom type="box" pos="25 1.8 1.4" size=".04 .4 .65" rgba=".12 .13 .15 1"/>
    <geom type="sphere" pos="24.95 1.8 1.4" size=".06" material="beacon"/>'''
        floor_appearance = 'material="floor"'
    else:
        scene_meshes = field_scene.xml_assets()
        scene_geoms = field_scene.xml_geoms()
        # 保留 Z=0 的原碰撞平面，但使其透明，避免盖住 CAD 地板或闪烁。
        floor_appearance = 'rgba="0.24 0.28 0.31 0"'
    # onboard 的 pos/xyaxes 都相对父物体 dart；默认在重心前方 0.23 米，
    # CAD 模式按镖体清单的安装位置放置，仍是理想针孔相机而非 OV7725 标定模型。
    # MuJoCo 相机沿自身 -Z 看、+Y 朝上：这里令相机右方为镖体 -Y、
    # 相机上方为镖体 +Z，因此默认沿镖体 +X 前视。它会随父物体转动。
    guide_light_assets, guide_light = "", ""
    if launch_profile is not None:
        lamp = _guide_light_metadata(launch_profile, guide_light_brightness)
        attributes = {"name": "guide_light", "pos": " ".join(str(value) for value in lamp["center_W_m"]),
                      "material": "guide_beacon", "contype": "0", "conaffinity": "0",
                      "group": "2", "density": "0"}
        if launch_profile.guide_light is not None:
            # 原 STL 穹顶已经按世界轴保存，局部原点为灯孔中心；只平移，
            # 不按靶板法向旋转，也不把量测口径当作网格缩放参数。
            asset = ET.Element("mesh", name="guide_light_mesh",
                               file=launch_profile.guide_light.mesh_file.as_posix(), inertia="shell")
            guide_light_assets = ET.tostring(asset, encoding="unicode")
            attributes.update(type="mesh", mesh="guide_light_mesh")
        else:
            attributes.update(type="sphere", size=str(lamp["radius_m"]))
        guide_light = ET.tostring(ET.Element("geom", attributes), encoding="unicode")
    # 原始活动模块已从静态场地分离，所有顶点仍沿世界轴、使用世界原点。
    # 将灯、外观与接触板放在同一台体，避免只移动亮点或留下静态重影。
    target_meshes = (getattr(field_scene, "xml_target_assets", lambda: "")()
                     if field_scene is not None else "")
    target_visuals = (getattr(field_scene, "xml_target_geoms", lambda: "")()
                      if field_scene is not None else "")
    target_stage = ""
    if guide_light or target_geom or target_visuals:
        drive = ""
        if launch_profile is not None and target_mode != "fixed":
            center = " ".join(str(value) for value in launch_profile.target_position_W_m)
            # 理想电机台：解析轨迹逐细步规定位置/速度；大惯量使碰撞反力
            # 对台体轨迹的偏移可忽略，同时让接触求解读取真实非零关节速度。
            drive = f'''<joint name="target_slide" type="slide" axis="0 1 0" damping="0"/>
                <inertial pos="{center}" mass="1000000000" diaginertia="1000000000 1000000000 1000000000"/>'''
        target_stage = (f'<body name="target_stage" gravcomp="1">{drive}'
                        f'{target_visuals}{guide_light}{target_geom}</body>')
    return f'''<mujoco model="dart_observer_prototype">
  <!-- MJCF 场景说明：长度用米，质量用千克；下面角度配置采用度。 -->
  <compiler angle="degree"/>
  <!-- timestep 是仿真时间步长；gravity 是世界 XYZ 方向的重力加速度。 -->
  <option timestep="{effective_dt}" gravity="0 0 -9.80665" integrator="RK4"/>
  <visual>
    <global offwidth="800" offheight="600"/>
    <quality shadowsize="512" offsamples="0"/>
    <!-- 该比例乘以整场地 extent；0.01 在2027场地会裁掉约0.84m内的近景。 -->
    <map znear="0.0001" zfar="100"/>
  </visual>
  <asset>
    <!-- asset 声明可复用的材质、纹理和网格，本身尚未在世界中摆放物体。 -->
    <texture name="sky" type="skybox" builtin="gradient" width="256" height="1536"
             rgb1="0.26 0.38 0.50" rgb2="0.76 0.83 0.89"/>
    <texture name="grid" type="2d" builtin="checker" width="256" height="256"
             rgb1="0.24 0.28 0.31" rgb2="0.29 0.33 0.36"/>
    <material name="floor" texture="grid" texrepeat="28 15" texuniform="true"/>
    <material name="beacon" rgba="0.1 1 0.15 1" emission="0.7"/>
    <material name="guide_beacon" rgba="0.02 0.45 0.04 1" emission="{max(0., min(1., float(guide_light_brightness)))}"/>
    {dart_meshes}
    {scene_meshes}
    {guide_light_assets}
    {target_meshes}
  </asset>
  <worldbody>
    <!-- worldbody 放置场地与物体；场地 OBJ 的顶点已经是米制世界坐标。 -->
    <light pos="8 -5 12" dir="0.2 0.3 -1" diffuse="0.9 0.9 0.9"/>
    <light pos="22 5 8" dir="-0.2 -0.3 -1" diffuse="0.6 0.6 0.6"/>
    <geom name="floor" type="plane" pos="14 0 0" size="18 10 .1" {floor_appearance}/>
    {scene_geoms}
    <body name="dart" pos="{position_text}">
      <!-- freejoint 允许镖体自由平移和转动；局部原点就是采用的估计重心。 -->
      <freejoint name="dart_free"/>
      <!-- inertial 独立配置物性，不从可视网格自动推断质量。 -->
      <inertial pos="0 0 0" mass="{config['mass_kg']}"
                {inertia}/>
      {dart_geoms}
      {nose_geom}
      <!-- 相机固连在镖体上：局部安装位置不变，世界位置随镖体运动。 -->
      <camera name="onboard" pos="{camera_position}" xyaxes="{camera_axes}"
              fovy="{config['camera_fov_deg']}"/>
    </body>
    <!-- 目标body位于镖体之后，保证镖体freejoint仍占最前7个位置/6个速度。 -->
    {target_stage}
  </worldbody>
</mujoco>'''


class DemoSimulation:
    """持有世界说明 model 和当前状态 data，统一提供运动、相机和重置。"""

    def __init__(self, config, scene_manifest=None, dart_manifest=None, launch_profile=None, *,
                 target_mode="fixed", target_seed=2027, aero_model=None,
                 flight_control_enabled=False, flight_control_config=None):
        """加载独立场地、镖体和发射资料；路径不混入纯数字参数表。

        首次启动以镖体清单物性、发射方案角度/速度覆盖数字配置。此后
        reset(config) 尊重调用方修改，避免 UI 或求解器参数被方案重复覆盖。
        不提供场地时也能运行相同镖体和发射状态，便于快速计算参数。
        """
        # 后端独立于数字配置；未知名称在加载网格/编译 MuJoCo 前拒绝。
        self.aero_model = resolve_aero_model(aero_model)
        if not isinstance(flight_control_enabled, bool):
            raise ValueError("飞控开关必须为bool。")
        self.flight_control_enabled = flight_control_enabled
        self.flight_config = load_flight_config(flight_control_config)
        self._external_renderer = None
        self._onboard_renderer = None
        self._graphics_renderer = None
        self.field_scene = load_field_scene(scene_manifest) if scene_manifest is not None else None
        self.dart_model = load_dart_model(dart_manifest) if dart_manifest is not None else None
        self.launch_profile = load_launch_profile(launch_profile) if launch_profile is not None else None
        self.target_mode, self.target_seed = target_mode, target_seed
        self.target_motion = TargetMotion(mode=target_mode, seed=target_seed)
        # 有发射方案时先看清站台里的镖体；场地独立预览仍先展示全景。
        self.external_view = ("overview" if self.field_scene is not None and self.launch_profile is None
                              else "base" if self.launch_profile is not None else "follow")
        initial = dict(config)
        if self.dart_model is not None:
            initial.update(mass_kg=self.dart_model.mass_kg,
                           reference_length_m=self.dart_model.reference_length_m,
                           reference_area_m2=self.dart_model.reference_area_m2,
                           camera_fov_deg=self.dart_model.camera_fovy_deg)
        if self.launch_profile is not None:
            initial.update(launch_height_m=self.launch_profile.position_W_m[2],
                           pitch_deg=self.launch_profile.pitch_deg,
                           yaw_deg=self.launch_profile.yaw_deg,
                           speed_mps=self.launch_profile.speed_mps)
        self.reset(initial)

    @property
    def t(self):
        """当前仿真时间，与真实电脑已经运行多久分开计算。"""
        return float(self.data.time)

    def reset(self, config=None, *, target_mode=None, target_seed=None,
              flight_control_enabled=None):
        """重新编译世界并恢复初始状态；先成功建好，再替换旧仿真。

        镖体自由关节固定占 qpos 前 7 项，前 3 项是世界位置，
        后 4 项是姿态四元数 w,x,y,z；qvel 共 6 项，前 3 项是世界
        线速度，后 3 项是自由关节的局部角速度。本次初始角速度保持零。
        移动靶的单滑轨关节放在镖体之后，位置/速度地址从模型读取。
        """
        # 首次构造还没有外部相机；后续应用参数时保留用户已调好的构图。
        camera_state = self.capture_camera() if hasattr(self, "external_camera") else None
        candidate = validate_config(config if config is not None else self.cfg)
        flight_enabled = (self.flight_control_enabled if flight_control_enabled is None
                          else flight_control_enabled)
        if not isinstance(flight_enabled, bool):
            raise ValueError("飞控开关必须为bool。")
        flight_controller = FlightController(self.flight_config)
        servo_actuation = ServoActuation(self.flight_config)
        imu_noise = ImuNoise(self.flight_config)
        mode = self.target_mode if target_mode is None else target_mode
        seed = self.target_seed if target_seed is None else target_seed
        motion = TargetMotion(mode=mode, seed=seed)
        if mode != "fixed" and self.launch_profile is None:
            raise ValueError("移动靶需要指定发射方案和目标平面。")
        # 方案固定发射 X/Y；Z 对应已有 launch_height_m，允许后续调整高度。
        launch = ([1., 0., candidate["launch_height_m"]] if self.launch_profile is None else
                  [*self.launch_profile.position_W_m[:2], candidate["launch_height_m"]])
        # model 保存结构和参数；data 保存位置、速度、时间等当前记录。
        model = mujoco.MjModel.from_xml_string(build_xml(candidate, self.field_scene, self.dart_model, launch,
            suppress_demo_props=self.launch_profile is not None, launch_profile=self.launch_profile,
            guide_light_brightness=candidate.get("guide_light_brightness", .8), target_mode=mode))
        data = mujoco.MjData(model)
        # 先成功建立候选模型，再释放旧模型的渲染器，保证错误可恢复。
        self.close()
        self.cfg, self.model, self.data = candidate, model, data
        self.flight_control_enabled = flight_enabled
        self.flight_controller, self.servo_actuation = flight_controller, servo_actuation
        self.imu_noise = imu_noise
        self.target_mode, self.target_seed, self.target_motion = mode, seed, motion
        # dt 是界面每次推进的宏步长；物理内核会在接触附近自动细分。
        self.dt = float(candidate["dt_s"])
        self.effective_dt = self.dt
        self.body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "dart")
        self._target_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "target_collision")
        self._floor_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self._target_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target_stage")
        slide = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "target_slide")
        self._target_qpos = int(model.jnt_qposadr[slide]) if slide >= 0 else None
        self._target_dof = int(model.jnt_dofadr[slide]) if slide >= 0 else None
        # Python 三角函数使用弧度，输入框使用度，因此先转换单位。
        yaw, pitch = np.deg2rad([candidate["yaw_deg"], candidate["pitch_deg"]])
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        # 该四元数使局部 +X 前轴指向设定仰角/偏航；仰角向上为正。
        self.data.qpos[3:7] = [cy*cp, sy*sp, -cy*sp, sy*cp]
        # An optional release roll rotates the body/camera about its own forward
        # axis without changing the launch velocity direction. Zero preserves
        # the original quaternion assignment exactly.
        roll_deg = candidate.get("roll_deg", 0.)
        if roll_deg != 0.:
            cr, sr = math.cos(math.radians(roll_deg)/2), math.sin(math.radians(roll_deg)/2)
            qw, qx, qy, qz = self.data.qpos[3:7].copy()
            self.data.qpos[3:7] = [qw*cr-qx*sr, qw*sr+qx*cr, qy*cr+qz*sr, qz*cr-qy*sr]
        # 将初速度大小分解到世界 XYZ；当前简化模型让速度与镖体前轴一致。
        self.data.qvel[:3] = candidate["speed_mps"] * np.array([
            math.cos(yaw)*math.cos(pitch), math.sin(yaw)*math.cos(pitch), math.sin(pitch)])
        self._sync_target_motion(forward=False)
        mujoco.mj_forward(model, data)  # 写入初始状态后同步派生位置/相机，不推进时间。
        self.done, self.status = False, "paused"
        self.internal_step_count = 0
        self.macro_step_count = 0
        self.target_result = None
        self.target_contact = None
        self._was_target_contact = False
        self._ground_settle_s = 0.
        self.last_aero_force = np.zeros(3)
        self.last_aero_torque = np.zeros(3)
        self.last_aero_diagnostics = None  # 尚未进行载荷计算；不伪造初始载荷。
        self._flight_specific_force_W = np.zeros(3)
        self._last_control_force_W = np.zeros(3)
        self._last_control_torque_W = np.zeros(3)
        self._last_visual_measurement = None
        rotation = self.data.xmat[self.body_id].reshape(3, 3)
        self._flight_yaw_reference_deg = float(np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0])))
        self.trajectory = [self.data.xpos[self.body_id].copy()]
        self.external_camera = mujoco.MjvCamera()
        self.external_camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.set_external_view(self.external_view)
        if camera_state is not None:
            self.restore_camera(camera_state)
        return self.observation()

    def set_target_mode(self, mode, seed=None):
        """先校验并建好新世界，再切换规则模式；无效模式不破坏当前仿真。"""
        return self.reset(target_mode=mode, target_seed=self.target_seed if seed is None else seed)

    def set_flight_control(self, enabled):
        """切换后从离架点开始新一轮，保留第三视角，防止混用两种实验状态。"""
        return self.reset(flight_control_enabled=enabled)

    def set_imu_noise(self, enabled, scale):
        """修改噪声开关/倍率后重开本轮；无效参数不破坏旧实验。"""
        return self.apply_flight_config({"imu_noise": {"enabled": enabled, "scale": scale}})

    def apply_flight_config(self, config_or_path):
        """应用飞控参数并重开本轮，保留开关、场地缓存与第三视角。

        映射按节覆盖当前生效值；路径加载文件并补齐默认值。候选仿真复用
        已加载的场地、镖体和气动对象，但隔离模型、动态与渲染器，失败时
        不替换当前实验。成功后才关闭旧渲染器并提交整个候选状态。
        """
        if isinstance(config_or_path, Mapping):
            patched = copy.deepcopy(self.flight_config)
            for section, values in config_or_path.items():
                if isinstance(values, Mapping) and isinstance(patched.get(section), dict):
                    patched[section].update(copy.deepcopy(dict(values)))
                else:
                    patched[section] = copy.deepcopy(values)
            parameters = load_flight_config(patched)
        else:
            parameters = load_flight_config(config_or_path)
        candidate = copy.copy(self)
        candidate.flight_config = parameters
        # copy.copy shares immutable asset caches, never live rendering resources.
        candidate._external_renderer = None
        candidate._onboard_renderer = None
        candidate._graphics_renderer = None
        try:
            observation = candidate.reset()
        except Exception:
            candidate.close()
            raise
        self.close()
        self.__dict__ = candidate.__dict__
        return observation

    def _flight_imu(self, velocity=None):
        """白名单理想自状态，绝不传目标平面/位置/运动状态给飞控。"""
        if velocity is None:
            velocity = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                    self.body_id, velocity, 0)
        result = imu_from_state(self.data.xmat[self.body_id].reshape(3, 3), velocity[:3],
            self._flight_specific_force_W*self.cfg["mass_kg"], self.cfg["mass_kg"])
        # 对齐imuTask的启动yaw归零；pitch/roll仍为绝对融合角。
        result["yaw_deg"] = ((result["yaw_deg"]-self._flight_yaw_reference_deg+180.) % 360.)-180.
        return self.imu_noise.measure(result, self.t)

    def submit_visual_measurement(self, details):
        """仅接收真实识别结果；相机CV系经安装外参转局部系，再提交控制器。"""
        if not self.flight_control_enabled:
            return None
        camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "onboard")
        rotation_WL = self.data.xmat[self.body_id].reshape(3, 3)
        rotation_Wcam = self.data.cam_xmat[camera_id].reshape(3, 3)
        rotation_LC = rotation_WL.T @ rotation_Wcam @ np.diag([1., -1., -1.])
        measurement = vision_from_detection(details, self.t, rotation_LC,
                                             float(self.model.cam_fovy[camera_id]))
        imu = self._flight_imu()
        self.flight_controller.submit_visual(measurement["t_s"], measurement["los_pitch_deg"],
            measurement["los_yaw_deg"], imu["pitch_rate_deg_s"], imu["yaw_rate_deg_s"],
            aim_pitch_deg=measurement["aim_pitch_deg"], aim_yaw_deg=measurement["aim_yaw_deg"],
            rotation_WL=rotation_from_imu(imu, self._flight_yaw_reference_deg) if measurement["valid"] else None,
            los_ray_L=measurement.get("los_ray_L"))
        self._last_visual_measurement = copy.deepcopy(measurement)
        return measurement

    @property
    def flight_control_metadata(self):
        result = copy.deepcopy(self.flight_controller.metadata())
        result.update(enabled=self.flight_control_enabled,
            sensor_adapter={"imu": "理想自姿态/角速度；差分加速度减重力得到含接触的重心比力",
                "gyro_channels": "pitch=-omega_L.y; yaw=omega_L.z，输入deg/s",
                "overload_channels": "pitch=-specific_force_L.z/g; yaw=-specific_force_L.y/g",
                "camera": "实际320x240内参及安装外参；检测质心与红色瞄准参考分别反投影；不使用目标真值",
                "los_rotation_compensation": "相邻图像实测光线按自身IMU完整姿态传输，消除包括滚转的3D旋转；姿态噪声保留",
                "aim": "红点角误差通过独立可调过载外环叠加PNG；光心保持(160,120)，橙点仅显示",
                "yaw": "启动自姿态yaw归零；pitch/roll保留绝对姿态角"},
            servo_model=copy.deepcopy(self.flight_config["servo"]),
            actuation_model={"name": "estimated_aggregate_control_derivatives",
                "parameters": copy.deepcopy(self.flight_config["actuation"]),
                "classification": "unvalidated_engineering_estimate",
                "positive_channels": "pitch抬头、yaw向左、roll绕局部X正向",
                "limitations": "聚合偏舵增量，不是逐CAD舵面气动；舵面外观暂未活动，未标定实机接线"},
            imu_noise=self.imu_noise.metadata())
        return result

    def _flight_observation(self):
        control = self.flight_controller.observation()
        servo = self.servo_actuation.observation()
        # 纯读取：显示不能调用有采样/RNG状态的_flight_imu。
        rotation = self.data.xmat[self.body_id].reshape(3, 3)
        specific_local = rotation.T @ self._flight_specific_force_W
        overload = {"pitch": float(-specific_local[2]/9.80665),
                    "yaw": float(-specific_local[1]/9.80665)}
        body_yaw = float(np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0])))
        velocity = self.data.qvel[:3]  # The dart free joint stores world linear velocity first.
        trajectory_yaw = float(np.degrees(np.arctan2(velocity[1], velocity[0])))
        motion = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                self.body_id, motion, 0)
        omega = rotation.T @ motion[:3]
        yaw_relative = ((body_yaw-self._flight_yaw_reference_deg+180.) % 360.)-180.
        attitude = {"pitch": float(np.degrees(np.arctan2(rotation[2, 0],
                              np.hypot(rotation[0, 0], rotation[1, 0])))),
                    "yaw": yaw_relative,
                    "roll": float(np.degrees(np.arctan2(rotation[2, 1], rotation[2, 2])))}
        rates = dict(pitch=float(np.degrees(-omega[1])),
                     yaw=float(np.degrees(omega[2])), roll=float(np.degrees(omega[0])))
        def demix(values):
            a, b, c, d = (float(v) for v in values)
            return dict(pitch=(a+b-c-d)/4., yaw=(a-b+c-d)/4., roll=(a-b-c+d)/4.)
        servo_cfg = self.flight_config["servo"]
        requested = np.asarray(control["command_deg"], dtype=float)
        actual_fin = np.asarray(servo["actual_deg"], dtype=float)
        held_fin = np.asarray(servo["held_deg"], dtype=float)
        pwm = np.asarray(servo["pwm_us"], dtype=float)
        mechanical = float(servo_cfg["limit_deg"])
        # Match ServoActuation: raw mixed request -> PWM clamp -> mechanical
        # held-angle clamp. A pending request need not be the last sampled PWM.
        pulse_raw = servo_cfg["center_us"]+servo_cfg["us_per_deg"]*requested
        pulse = np.clip(pulse_raw, servo_cfg["pwm_min_us"], servo_cfg["pwm_max_us"])
        clipped = np.clip((pulse-servo_cfg["center_us"])/servo_cfg["us_per_deg"],
                          -mechanical, mechanical)
        relative_velocity = np.asarray(velocity)-np.array([0., self.cfg["wind_cross_mps"], 0.])
        actuator = dict(available=True, servo_limit_deg=mechanical,
            request_diagnostics_scope="current raw mixed request; held PWM is sampled separately",
            request_time_s=self.t,
            held_pwm_time_s=((servo["next_pwm_index"]-1)*servo_cfg["pwm_period_s"]
                             if servo["next_pwm_index"] else None),
            mixed_command_exceeds_limit=(np.abs(requested) > mechanical+1e-10).tolist(),
            command_clipped_deg=clipped.tolist(),
            pwm_limited=((pulse_raw < servo_cfg["pwm_min_us"]-1e-10) |
                         (pulse_raw > servo_cfg["pwm_max_us"]+1e-10)).tolist(),
            pwm_at_limit=((pwm <= servo_cfg["pwm_min_us"]+1e-10) |
                          (pwm >= servo_cfg["pwm_max_us"]-1e-10)).tolist(),
            held_at_mechanical_limit=(np.abs(held_fin) >= mechanical-1e-10).tolist(),
            actual_at_mechanical_limit=(np.abs(actual_fin) >= mechanical-1e-10).tolist())
        return {"enabled": self.flight_control_enabled, "realization": "physical",
                "attitude_deg": attitude, "angular_rate_deg_s": rates,
                "attitude_reference": "yaw relative to launch; pitch/roll absolute",
                "actual_specific_force_L_g": dict(zip(("x", "y", "z"),
                                                       (specific_local/9.80665).tolist())),
                "actual_time_s": self.t,
                "axis_command_deg": demix(requested if self.flight_control_enabled else np.zeros(4)),
                "axis_actual_deg": demix(actual_fin),
                "servo_held_deg": held_fin.tolist(),
                "actuator_diagnostics": actuator,
                "dynamic_pressure_pa": float(.5*self.cfg["rho_kg_m3"]*
                                               np.dot(relative_velocity, relative_velocity)),
                "command_limit_kind": "per_axis", 
                "command_limit_g": float(self.flight_config["controller"]["max_accel_g"]),
                "autopilot_used": self.flight_control_enabled, "servo_model_used": self.flight_control_enabled,
                "yaw_attitude_relative_deg": ((body_yaw-self._flight_yaw_reference_deg+180.) % 360.)-180.,
                "yaw_trajectory_relative_deg": ((trajectory_yaw-self._flight_yaw_reference_deg+180.) % 360.)-180.,
                "status": control["tracking_status"] if self.flight_control_enabled else "disabled",
                "servo_command_deg": list(control["command_deg"]),
                "servo_actual_deg": list(servo["actual_deg"]),
                "servo_pwm_us": list(servo["pwm_us"]),
                "control": control,
                "actual_overload_g": overload,
                "overload_time_s": self.t,
                "overload_convention": "pitch down positive; yaw right positive; non-gravity specific force includes contact",
                "imu_noise": self.imu_noise.observation(),
                "visual_measurement": copy.deepcopy(self._last_visual_measurement),
                "control_force_W_N": self._last_control_force_W.tolist(),
                "control_torque_W_Nm": self._last_control_torque_W.tolist()}

    def target_position_W(self, t=None):
        """名义靶心沿世界Y滑轨平移；灯具、网格和碰撞板使用同一位移。"""
        if self.launch_profile is None:
            return None
        state = self.target_motion.sample(self.t if t is None else t)
        return np.asarray(self.launch_profile.target_position_W_m) + [0., state["offset_m"], 0.]

    def _sync_target_motion(self, t=None, *, forward=True):
        """规定理想电机台状态；关节速度参与MuJoCo接触相对速度计算。"""
        if self._target_qpos is None:
            return
        state = self.target_motion.sample(self.t if t is None else t)
        self.data.qpos[self._target_qpos] = state["offset_m"]
        self.data.qvel[self._target_dof] = state["velocity_mps"]
        if forward:
            mujoco.mj_forward(self.model, self.data)

    def capture_camera(self):
        """复制第三视角构图；与镖体、物理状态及第一视角相机分开保存。"""
        camera = self.external_camera
        return {"lookat": camera.lookat.copy(), "distance": float(camera.distance),
                "azimuth": float(camera.azimuth), "elevation": float(camera.elevation)}

    def restore_camera(self, state):
        """恢复用户的观察中心、旋转与缩放，不调用物理积分。"""
        self.external_camera.lookat[:] = state["lookat"]
        for key in ("distance", "azimuth", "elevation"):
            setattr(self.external_camera, key, float(state[key]))

    def capture_state(self):
        """复制当前运行状态，供本轮回放使用，不保存或重建场地模型。

        INTEGRATION 包含位置、速度、时间以及积分器所需状态；接触记录、
        演示状态和显示轨迹属于 Python 侧，另行复制以便回放读数一致。
        """
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        integration = np.empty(mujoco.mj_stateSize(self.model, spec))
        mujoco.mj_getState(self.model, self.data, integration, spec)
        return {"integration": integration, "done": self.done, "status": self.status,
                "target_result": copy.deepcopy(self.target_result),
                "target_contact": copy.deepcopy(self.target_contact),
                "was_target_contact": self._was_target_contact,
                "ground_settle_s": self._ground_settle_s,
                "force": self.last_aero_force.copy(), "torque": self.last_aero_torque.copy(),
                "aero_diagnostics": copy.deepcopy(self.last_aero_diagnostics),
                "trajectory": np.asarray(self.trajectory).copy(),
                "target_motion": self.target_motion.capture_state(),
                "flight_control": {
                    "enabled": self.flight_control_enabled,
                    "controller": self.flight_controller.capture_state(),
                    "servo": self.servo_actuation.capture_state(),
                    "imu_noise": self.imu_noise.capture_state(),
                    "specific_force_W": self._flight_specific_force_W.copy(),
                    "control_force_W": self._last_control_force_W.copy(),
                    "control_torque_W": self._last_control_torque_W.copy(),
                    "visual_measurement": copy.deepcopy(self._last_visual_measurement),
                    "yaw_reference_deg": self._flight_yaw_reference_deg},
                "internal_step_count": self.internal_step_count,
                "macro_step_count": self.macro_step_count}

    def restore_state(self, state):
        """恢复同一模型的一帧；mj_forward 只更新派生量，不让物体继续运动。"""
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        integration = np.asarray(state["integration"], dtype=float)
        if integration.shape != (mujoco.mj_stateSize(self.model, spec),):
            raise ValueError("回放状态与当前模型不匹配，请重新运行后再回放。")
        flight_state = state.get("flight_control")
        controller = servo = noise = None
        if flight_state is not None:
            if flight_state["enabled"] != self.flight_control_enabled:
                raise ValueError("回放飞控模式与当前实验不匹配，请重新录制。")
            controller = FlightController(self.flight_config)
            servo = ServoActuation(self.flight_config)
            noise = ImuNoise(self.flight_config)
            controller.restore_state(flight_state["controller"])
            servo.restore_state(flight_state["servo"])
            noise.restore_state(flight_state["imu_noise"])
        elif self.flight_control_enabled:
            raise ValueError("回放缺少飞控与执行机构状态。")
        if "target_motion" in state:
            self.target_motion.restore_state(state["target_motion"])
        elif self.target_mode != "fixed":
            raise ValueError("移动靶回放缺少目标运动事件。")
        mujoco.mj_setState(self.model, self.data, integration, spec)
        self.done, self.status = bool(state["done"]), state["status"]
        self.target_result = copy.deepcopy(state["target_result"])
        self.target_contact = copy.deepcopy(state["target_contact"])
        self._was_target_contact = state["was_target_contact"]
        self._ground_settle_s = state["ground_settle_s"]
        self.last_aero_force = np.asarray(state["force"]).copy()
        self.last_aero_torque = np.asarray(state["torque"]).copy()
        self.last_aero_diagnostics = copy.deepcopy(state.get("aero_diagnostics"))
        if flight_state is not None:
            self.flight_controller, self.servo_actuation = controller, servo
            self.imu_noise = noise
            self._flight_specific_force_W = np.asarray(flight_state["specific_force_W"]).copy()
            self._last_control_force_W = np.asarray(flight_state["control_force_W"]).copy()
            self._last_control_torque_W = np.asarray(flight_state["control_torque_W"]).copy()
            self._last_visual_measurement = copy.deepcopy(flight_state["visual_measurement"])
            self._flight_yaw_reference_deg = float(flight_state["yaw_reference_deg"])
        self.trajectory = [row.copy() for row in state["trajectory"]]
        self.internal_step_count = state["internal_step_count"]
        self.macro_step_count = state["macro_step_count"]
        self.model.opt.timestep = self.effective_dt = self.dt
        self._sync_target_motion(forward=False)
        mujoco.mj_forward(self.model, self.data)

    @property
    def aero_metadata(self):
        """保存所选模型及生效参数，不将估计参数当作 CFD 或实测数据。"""
        return copy.deepcopy(self.aero_model.metadata(self.cfg))

    @property
    def dart_metadata(self):
        """记录真正生效的质量和惯量，保留 CAD 几何估计的来源。"""
        if self.dart_model is None:
            return {"mode": "simplified_demo", "mass_kg": self.cfg["mass_kg"],
                    "provenance": "演示质量与简化惯量；未采用真实镖体物性"}
        result = self.dart_model.metadata()
        if self.target_collision_enabled:
            result["collision"] = "独立短圆柱触发头 + 前端后缩的镖身胶囊；仅近似刚体碰撞，不模拟翼面碰撞/材料破坏"
        result["nominal_mass_kg"] = result["mass_kg"]
        result["mass_kg"] = self.cfg["mass_kg"]
        ratio = self.cfg["mass_kg"] / self.dart_model.mass_kg
        result["full_inertia_kg_m2"] = [value * ratio for value in self.dart_model.full_inertia]
        result["reference_length_m"] = self.cfg["reference_length_m"]
        result["reference_area_m2"] = self.cfg["reference_area_m2"]
        result["mass_change_assumption"] = "改变质量时按相同比例缩放转动惯量，内部质量分布不变"
        return result

    @property
    def launch_metadata(self):
        """返回当前实际生效的起点、角度和速度，区别于名义发射方案。"""
        position = ([1., 0., self.cfg["launch_height_m"]] if self.launch_profile is None else
                    [*self.launch_profile.position_W_m[:2], self.cfg["launch_height_m"]])
        actual = {"position_W_m": position, "pitch_deg": self.cfg["pitch_deg"],
                  "yaw_deg": self.cfg["yaw_deg"], "roll_deg": self.cfg.get("roll_deg", 0.),
                  "speed_mps": self.cfg["speed_mps"]}
        if self.launch_profile is None:
            return {"mode": "simplified_demo", "launch": actual}
        result = self.launch_profile.metadata()
        result["nominal_launch"] = result["launch"]
        result["launch"] = actual
        result["guide_light"] = _guide_light_metadata(
            self.launch_profile, self.cfg.get("guide_light_brightness", .8))
        result["guide_light"]["nominal_center_W_m"] = result["guide_light"]["center_W_m"]
        result["guide_light"]["center_W_m"] = (np.asarray(
            result["guide_light"]["nominal_center_W_m"]) +
            [0., self.target_motion.sample(self.t)["offset_m"], 0.]).tolist()
        result["target"]["nominal_position_W_m"] = result["target"]["position_W_m"]
        result["target"]["position_W_m"] = self.target_position_W().tolist()
        result["target_motion"] = self.target_motion.metadata()
        result["target_motion"]["physics"] = {
            "model": "固定靶为静态body；移动靶为世界Y滑轨单自由度理想电机台",
            "prescribed_state": "每个内部物理步前后按解析轨迹规定滑轨qpos/qvel；非实测电机控制",
            "stage_mass_kg": 1e9 if self.target_mode != "fixed" else None,
            "relative_velocity": "移动靶滑轨速度参与MuJoCo接触求解；不使用零速度mocap伪装移动碰撞",
            "limitations": "大惯量台近似强制轨迹；未模拟真实电机、支架柔性或受击后的台体退让",
        }
        result["integration"] = {"requested_dt_s": self.cfg["dt_s"],
                                 "macro_dt_s": self.dt,
                                 "contact_dt_limit_s": .0002 if self.launch_profile is not None else None,
                                 "method": "远离接触区域使用宏步长；靶板/地面附近自动细分"}
        return result

    @property
    def target_collision_enabled(self):
        """当前方案是否启用真实积分的靶板接触，供窗口及数值求解器查询。"""
        return self.launch_profile is not None and self.launch_profile.target_collision.enabled

    @property
    def scene_metadata(self):
        """记录当前使用哪份场地；截图状态文件会保存这项信息。"""
        if self.field_scene is None:
            return {"mode": "simplified_demo", "source": {"name": "简化演示场景", "season": None}}
        return self.field_scene.metadata()

    @property
    def scene_label(self):
        """面向窗口用户的场景说明。"""
        return self.field_scene.label if self.field_scene is not None else "简化演示场景 · 未加载场地 CAD"

    def set_external_view(self, view):
        """选择全景、跟随或基地后方预设；切换相机不推进仿真时间。"""
        if view not in ("follow", "overview", "base"):
            raise ValueError("外部视角必须是 follow、overview 或 base。")
        self.external_view = view
        if view == "overview":
            overview = self.field_scene.overview if self.field_scene is not None else {
                "lookat": [14., 0., 1.], "distance": 36., "azimuth": 135., "elevation": -45.}
            # lookat 是世界中的观察中心；距离、方位角、俯仰角共同确定外部相机。
            self.external_camera.lookat[:] = overview["lookat"]
            self.external_camera.distance = overview["distance"]
            self.external_camera.azimuth = overview["azimuth"]
            self.external_camera.elevation = overview["elevation"]
        elif view == "base" and self.launch_profile is not None:
            # 由蓝方基地后方朝发射架方向观察；距离保证能看见场地主要飞行段。
            launch = np.asarray(self.launch_profile.position_W_m)
            target = self.target_position_W()
            self.external_camera.lookat[:] = (launch + target) / 2
            self.external_camera.distance = 32.
            self.external_camera.azimuth = 5.
            self.external_camera.elevation = -24.
        else:
            self.external_camera.lookat[:] = self.data.xpos[self.body_id]
            if self.launch_profile is not None:
                # 发射站台背板会遮住旧的远距离跟随相机。靠近镖体并从开口方向
                # 观察，既能看清 CAD 外形，也不改变镖载相机或物理发射方向。
                self.external_camera.distance = .65
                self.external_camera.azimuth = self.cfg["yaw_deg"] + 12.5
                self.external_camera.elevation = -15
            else:
                self.external_camera.distance = 3.1
                self.external_camera.azimuth = 135
                self.external_camera.elevation = -18

    def observation(self):
        """把 MuJoCo 的当前记录整理成外部程序可以直接读取的状态。

        mj_objectVelocity 的六维输出顺序是 [角速度 XYZ, 线速度 XYZ]，
        和自由关节 qvel 的 [线速度, 角速度] 顺序不同。最后一个参数 0
        表示输出轴向使用世界坐标，因此 velocity[3:] 就是世界线速度。
        """
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                self.body_id, velocity, 0)
        # copy 防止外部修改返回数组时连带修改内部记录。
        return {"t": self.t, "p_W": self.data.xpos[self.body_id].copy(),
                "v_W": velocity[3:].copy(), "speed": float(np.linalg.norm(velocity[3:])),
                "altitude": float(self.data.xpos[self.body_id][2]),
                "done": self.done, "status": self.status,
                "aero_force_W": self.last_aero_force.copy(),
                "aero_torque_W": self.last_aero_torque.copy(),
                # 载荷取最近内部步积分前的状态，aero.t_s 是其独立时间戳。
                "aero": copy.deepcopy(self.last_aero_diagnostics),
                "flight_control": self._flight_observation(),
                "target_result": copy.deepcopy(self.target_result),
                "target_contact": copy.deepcopy(self.target_contact),
                "target_state": (copy.deepcopy(self.target_motion.sample(self.t))
                                 if self.launch_profile is not None else None),
                "target_center_W_m": (self.target_position_W().tolist()
                                      if self.launch_profile is not None else None),
                "effective_dt_s": self.effective_dt, "macro_dt_s": self.dt}

    def nose_position_W(self):
        """计算镖头的世界坐标，单位米；姿态变化会改变头部相对重心的位置。

        R 将镖体局部坐标转到世界坐标，因此 p_头 = p_重心 + R × r_局部头部。
        简化演示镖头取椭球最前端；CAD 模式使用清单给定的 nose_tip_m。
        """
        nose = [0.215, 0., 0.] if self.dart_model is None else self.dart_model.nose_tip
        rotation = self.data.xmat[self.body_id].reshape(3, 3)
        return self.data.xpos[self.body_id] + rotation @ np.asarray(nose)

    @staticmethod
    def _target_invalid_reason(geometric_hit, state):
        """分开说明几何偏靶与规则检测窗口未开放。"""
        if not geometric_hit:
            return "接触或过靶位置在判定半径外"
        if state["hit_eligible"]:
            return None
        if state.get("moving", False):
            return "移动靶尚在运动，本次几何触及不计有效命中"
        if not state.get("detection_open", True):
            return "目标检测窗口尚未开放或已关闭"
        return "本次移动或稳定等待阶段尚未完成"

    def _check_target_crossing(self, previous_tip, next_tip, previous_time, next_time):
        """检测镖头首次穿过靶平面，并在相邻时间步之间线性插值。

        法向量已朝向发射点到靶心；有向距离从负到正表示正向穿过。
        无碰撞旧方案判定后保存交点并停止；碰撞方案只记录交点，继续积分。
        均不移动镖体或吸附到靶心。物理接触可能使头部反弹而不穿过平面，
        此时应查看 target_contact，不把“没有过面”误当“没有碰到”。
        """
        # 碰撞已经改变轨迹后，后续翻转/落地穿过无限靶平面不再代表这次
        # 入射的自由飞行落点。保留已有过面记录，只用真实接触事件评价触板。
        if (self.launch_profile is None or self.target_result is not None
                or (self.target_collision_enabled and self.target_contact is not None)):
            return
        profile = self.launch_profile
        previous_center = self.target_position_W(previous_time)
        next_center = self.target_position_W(next_time)
        normal = np.asarray(profile.target_normal_W)
        distance_before = float((previous_tip - previous_center) @ normal)
        distance_after = float((next_tip - next_center) @ normal)
        if distance_before > 0 or distance_after < 0 or distance_after <= distance_before:
            return
        # 距离为零的比例落在 [0,1]；采用该比例同时插值位置和仿真时间。
        fraction = float(np.clip(-distance_before / (distance_after - distance_before), 0., 1.))
        crossing = previous_tip + fraction * (next_tip - previous_tip)
        event_time = float(previous_time + fraction * (next_time - previous_time))
        center = self.target_position_W(event_time)
        target_state = self.target_motion.sample(event_time)
        error = crossing - center
        in_plane_error = error - float(error @ normal) * normal
        miss = float(np.linalg.norm(in_plane_error))
        geometric_hit = miss <= profile.target_radius_m
        hit = geometric_hit and bool(target_state["hit_eligible"])
        self.target_result = {
            "crossing_time_s": event_time,
            "position_W_m": crossing.tolist(), "miss_distance_m": miss, "hit": hit,
            "geometric_hit": geometric_hit, "rule_eligible": bool(target_state["hit_eligible"]),
            "invalid_reason": self._target_invalid_reason(geometric_hit, target_state),
            "target_state": copy.deepcopy(target_state),
            "in_plane_error_W_m": in_plane_error.tolist(),
            "target_position_W_m": center.tolist(), "normal_W": normal.tolist(),
            "radius_m": profile.target_radius_m,
            "method": "镖头首次正向过动态靶平面，相邻物理步分别取靶心后线性插值；几何位置与规则窗口分别评价，非官方裁判判定",
        }
        if hit:
            self.target_motion.notify_hit(event_time)
            self._sync_target_motion()
        if not self.target_collision_enabled:
            self.status, self.done = ("target_hit" if hit else "target_miss"), True

    def _update_target_contact(self, previous_velocity_W):
        """记录首次实际靶板接触，并持续汇总接触力；不向物体施加任何额外作用。

        mj_contactForce 输出接触坐标中的力/力矩，第 0 项为法向力。本事件
        使用步末接触解，因此时间精度为有效步长；冲量是力×步长的离散估计。
        首次接触时间/位置保持不变，离开靶板时保存一次碰撞后重心速度。
        """
        if self._target_geom_id < 0:
            return
        samples = []
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            if self._target_geom_id not in (contact.geom1, contact.geom2) or contact.efc_address < 0:
                continue
            other = contact.geom2 if contact.geom1 == self._target_geom_id else contact.geom1
            if self.model.geom_bodyid[other] != self.body_id:
                continue
            wrench = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, index, wrench)
            if wrench[0] > 0:
                samples.append((other, np.array(contact.pos), float(wrench[0]), float(contact.dist)))
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                self.body_id, velocity, 0)
        if samples:
            # 多点同时出现时优先记录镖头；否则保留真实首先接触的镖身点。
            samples.sort(key=lambda item: (mujoco.mj_id2name(self.model,
                mujoco.mjtObj.mjOBJ_GEOM, item[0]) != "nose_contact", -item[2]))
            other, point, _, separation = samples[0]
            force = sum(item[2] for item in samples)
            if self.target_contact is None:
                target_state = self.target_motion.sample(self.t)
                center = self.target_position_W()
                normal = np.asarray(self.launch_profile.target_normal_W)
                error = point - center
                in_plane = float(np.linalg.norm(error - (error @ normal)*normal))
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, other)
                geometric_hit = in_plane <= self.launch_profile.target_radius_m
                hit = geometric_hit and bool(target_state["hit_eligible"])
                target_velocity = np.zeros(6)
                mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                        self._target_body_id, target_velocity, 0)
                relative_velocity = np.asarray(previous_velocity_W)-target_velocity[3:]
                self.target_contact = {
                    "time_s": self.t, "position_W_m": point.tolist(),
                    "nose_position_W_m": self.nose_position_W().tolist(),
                    "in_plane_distance_m": in_plane,
                    "geom_name": "nose" if name == "nose_contact" else name,
                    "physical_contact": True, "geometric_hit": geometric_hit,
                    "rule_eligible": bool(target_state["hit_eligible"]), "hit": hit,
                    "invalid_reason": self._target_invalid_reason(geometric_hit, target_state),
                    "target_state": copy.deepcopy(target_state),
                    "target_position_W_m": center.tolist(),
                    "target_velocity_W_mps": target_velocity[3:].tolist(),
                    "relative_pre_contact_velocity_W_mps": relative_velocity.tolist(),
                    "relative_pre_contact_normal_speed_mps": float(relative_velocity @ normal),
                    "pre_contact_velocity_W": np.asarray(previous_velocity_W).tolist(),
                    "post_contact_velocity_W": velocity[3:].tolist(),
                    "normal_force_N": force, "peak_normal_force_N": force,
                    "normal_impulse_Ns": 0., "contact_steps": 0,
                    "max_penetration_m": max(0., -separation),
                    "method": "MuJoCo 镖头/镖身与薄盒实际接触，移动台关节速度参与相对接触求解；规则窗口独立评价，软接触参数未实测，非官方比赛判定",
                }
                if hit:
                    self.target_motion.notify_hit(self.t)
                    self._sync_target_motion()
            event = self.target_contact
            event.update(last_contact_time_s=self.t, in_contact=True,
                         normal_force_N=force, post_contact_velocity_W=velocity[3:].tolist())
            event["peak_normal_force_N"] = max(event["peak_normal_force_N"], force)
            event["normal_impulse_Ns"] += force*self.effective_dt
            event["contact_steps"] += 1
            event["max_penetration_m"] = max(event["max_penetration_m"],
                                                max(-item[3] for item in samples))
        elif self.target_contact is not None:
            self.target_contact.update(in_contact=False, normal_force_N=0.)
            if self._was_target_contact:
                self.target_contact["post_contact_velocity_W"] = velocity[3:].tolist()
        self._was_target_contact = bool(samples)

    def _ground_is_settled(self):
        """碰撞模式让落地接触继续计算，低速稳定 0.15 秒后才结束。

        旧的重心高度阈值会截断触地前后的运动；这里以实际地面接触和
        线/角速度共同判断，超时仍由 t_max_s 兜底。
        """
        touching = any(self._floor_geom_id in (self.data.contact[index].geom1,
            self.data.contact[index].geom2) and self.data.contact[index].dist <= 0
            for index in range(self.data.ncon))
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                self.body_id, velocity, 0)
        slow = np.linalg.norm(velocity[3:]) < .12 and np.linalg.norm(velocity[:3]) < .5
        self._ground_settle_s = self._ground_settle_s+self.effective_dt if touching and slow else 0.
        return self._ground_settle_s >= .15

    def step(self, steps=1):
        """推进指定宏步数，并在靶板/地面附近自动缩小内部物理步长。

        steps 可以批量推进多步，但每一步仍重新读取姿态、计算空气作用。
        本方法不睡眠、不绘图；是否追赶真实时间由窗口的 _tick 决定。
        """
        for _ in range(steps):
            if self.done:
                break
            macro_end = min(self.t + self.dt, self.cfg["t_max_s"])
            geometric_stop_after_macro = False
            while not self.done and self.t < macro_end - 1e-12:
                self._sync_target_motion()
                if self.flight_control_enabled:
                    command_deg = self.flight_controller.advance_to(self.t, self._flight_imu())
                # 用镖头到靶平面的距离决定是否细算；低空时同样细算，防止
                # 镖体穿过地面。远离靶/地面的自由飞行按宏步长前进。
                micro_dt = self.dt
                if self.launch_profile is not None:
                    profile = self.launch_profile
                    signed_distance = abs(float((self.nose_position_W() -
                        self.target_position_W()) @ np.asarray(profile.target_normal_W)))
                    near_target = signed_distance < .25
                    near_ground = self.data.xpos[self.body_id][2] < .14
                    if near_target or near_ground:
                        micro_dt = min(self.dt, .0002)
                micro_dt = min(micro_dt, macro_end - self.t)
                if self.flight_control_enabled:
                    # 500Hz宏步与200Hz控制不整除，细分到每个5ms边界，避免控制抖动。
                    until_control = self.flight_controller.next_update_time_s-self.t
                    if until_control > 1e-12:
                        micro_dt = min(micro_dt, until_control)
                self.effective_dt = micro_dt
                self.model.opt.timestep = micro_dt
                obs = self.observation()
                previous_tip, previous_time = self.nose_position_W(), self.t
                velocity = np.zeros(6)
                mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                        self.body_id, velocity, 0)
                R = self.data.xmat[self.body_id].reshape(3, 3)
                state = AeroState(self.t, obs["v_W"], R, velocity[:3],
                                  np.array([0., self.cfg["wind_cross_mps"], 0.]))
                loads, diagnostics = evaluate_aerodynamics(self.aero_model, state, self.cfg)
                force, torque = loads.force_W_N, loads.torque_W_Nm
                if self.flight_control_enabled:
                    # 本步载荷只使用t时刻实际舵角，不提前使用t+dt的响应。
                    actual = np.asarray(self.servo_actuation.observation()["actual_deg"])
                    # none明确关闭全部气动，即使飞控仍运行也不能暗中添加舵效。
                    if loads.domain_status != "disabled":
                        increment = estimated_control_loads(state, self.cfg, actual, self.flight_config)
                        self._last_control_force_W = increment.force_W_N.copy()
                        self._last_control_torque_W = increment.torque_W_Nm.copy()
                        force = force+increment.force_W_N
                        torque = torque+increment.torque_W_Nm
                        diagnostics["details"]["control_increment"] = copy.deepcopy(increment.details)
                    else:
                        self._last_control_force_W = np.zeros(3)
                        self._last_control_torque_W = np.zeros(3)
                    diagnostics["force_W_N"] = force.tolist()
                    diagnostics["torque_W_Nm"] = torque.tolist()
                self.last_aero_force, self.last_aero_torque = force, torque
                self.last_aero_diagnostics = diagnostics
                self.data.xfrc_applied[:] = 0.
                self.data.xfrc_applied[self.body_id, :3] = force
                self.data.xfrc_applied[self.body_id, 3:] = torque
                mujoco.mj_step(self.model, self.data)
                self.internal_step_count += 1
                self._sync_target_motion(forward=False)
                mujoco.mj_forward(self.model, self.data)
                if self.flight_control_enabled:
                    self.servo_actuation.advance(command_deg, micro_dt)
                # 即使飞控关闭也记录实际比力；纯观测不改变力、积分或噪声序列。
                after_velocity = np.zeros(6)
                mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                        self.body_id, after_velocity, 0)
                self._flight_specific_force_W = ((after_velocity[3:]-velocity[3:])/micro_dt
                                                 - self.model.opt.gravity)
                position = self.data.xpos[self.body_id]
                self.status = "running"
                if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                    self.status, self.done = "invalid", True
                else:
                    self._update_target_contact(obs["v_W"])
                    self._check_target_crossing(previous_tip, self.nose_position_W(), previous_time, self.t)
                    if self.target_result is not None and not self.target_collision_enabled:
                        # 几何判定沿用“完成当前宏步再暂停”的行为，避免步数接口
                        # 因内部细分而改变；命中位置/时间仍按细步插值计算。
                        geometric_stop_after_macro = True
                        self.done = False
                        self.status = "running"
                    grounded = (self._ground_is_settled() if self.target_collision_enabled
                                else position[2] <= .03)
                    if not self.done and grounded:
                        self.status, self.done = "landed", True
                    elif not self.done and self.t >= self.cfg["t_max_s"] - 1e-12:
                        self.status, self.done = "finished", True
                if len(self.trajectory) == 0 or self.t >= (len(self.trajectory) * .02) - 1e-9:
                    self.trajectory.append(position.copy())
            self.model.opt.timestep = self.dt
            self.effective_dt = self.dt
            self.macro_step_count += 1
            if geometric_stop_after_macro:
                self.done = True
                self.status = "target_hit" if self.target_result["hit"] else "target_miss"
        return self.observation()

    @property
    def graphics_renderer(self):
        """返回当前 OpenGL 上下文实际报告的显卡名称。"""
        return self._graphics_renderer

    def _read_gl_renderer(self):
        """查询实际用于渲染的设备，不能仅凭机器型号推断。"""
        try:
            import ctypes
            import glfw
            pointer = glfw.get_proc_address("glGetString")
            get_string = ctypes.CFUNCTYPE(ctypes.c_char_p, ctypes.c_uint)(pointer)
            value = get_string(0x1F01)  # 当前 OpenGL 上下文中的 GL_RENDERER。
            self._graphics_renderer = value.decode("ascii", errors="replace") if value else "unknown"
        except Exception:
            self._graphics_renderer = "unknown"

    def render(self, camera="external", *, width=None, height=None):
        """从当前状态生成 RGB 图片；两台相机读取同一个世界和时间。

        update_scene 把物体状态变成可绘制场景，render 再生成像素；
        这里不调用 mj_step，因此拍照、切换视角都不会让镖体多飞一步。
        """
        if camera == "external":
            if (width is None) != (height is None):
                raise ValueError("External viewport requires both width and height")
            if width is not None:
                if (isinstance(width, bool) or isinstance(height, bool) or
                        not math.isfinite(float(width)) or not math.isfinite(float(height)) or
                        width <= 0 or height <= 0):
                    raise ValueError("External viewport dimensions must be positive finite numbers")
                scale = min(1., 1600./float(width), 1000./float(height))
                size = (max(1, round(float(width)*scale)), max(1, round(float(height)*scale)))
            else:
                size = getattr(self, "_external_renderer_size", (800, 600))
            if (self._external_renderer is not None and
                    getattr(self, "_external_renderer_size", (800, 600)) != size):
                self._close_renderer(self._external_renderer)
                self._external_renderer = None
            if self._external_renderer is None:
                # CAD 可能拆成许多 geom，场景容量同时给黄色轨迹预留空间。
                self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, size[0])
                self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, size[1])
                self._external_renderer = mujoco.Renderer(self.model, height=size[1], width=size[0],
                    max_geom=max(2000, self.model.ngeom + 1000))
                self._external_renderer_size = size
                self._read_gl_renderer()
            renderer = self._external_renderer
            if self.external_view == "follow":
                self.external_camera.lookat[:] = self.data.xpos[self.body_id]
            renderer.update_scene(self.data, camera=self.external_camera)
            # 黄色轨迹只用于显示，没有质量，也不会向镖体施力。
            # 全景加粗以便远处可见；近景用细线，避免遮住只有二十多厘米的镖体。
            line_width = .05 if self.external_view == "overview" else .0015
            for start, end in zip(self.trajectory[:-1], self.trajectory[1:]):
                if renderer.scene.ngeom >= renderer.scene.maxgeom:
                    break
                geom = renderer.scene.geoms[renderer.scene.ngeom]
                mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
                    np.zeros(3), np.zeros(3), np.eye(3).reshape(-1), np.array([1., .72, .15, 1.]))
                mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, line_width, start, end)
                renderer.scene.ngeom += 1
        elif camera == "onboard":
            if self._onboard_renderer is None:
                self._onboard_renderer = mujoco.Renderer(self.model, height=240, width=320,
                    max_geom=max(2000, self.model.ngeom + 32))
            renderer = self._onboard_renderer
            # 名称指向 XML 中装在镖体上的相机，父物体位姿决定相机世界变换。
            renderer.update_scene(self.data, camera="onboard")
        else:
            raise ValueError("Unknown camera: %s" % camera)
        return renderer.render().copy()  # 独立保存像素，避免下一次渲染覆盖调用方图片。

    @staticmethod
    def _close_renderer(renderer):
        # MuJoCo frees GPU objects in the current GL context.  A recent onboard
        # frame may have made the other context current; never delete its assets
        # when resizing or replacing the external renderer.
        context = getattr(renderer, "_gl_context", None)
        if context is not None:
            context.make_current()
        renderer.close()

    def close(self):
        """关闭两台相机的渲染资源；重复调用也可安全退出。"""
        for name in ("_external_renderer", "_onboard_renderer"):
            renderer = getattr(self, name, None)
            if renderer is not None:
                self._close_renderer(renderer)
                setattr(self, name, None)


def main(argv=None):
    """无窗口运行并打印结果；显式指定输出时只保存数值文件。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--scene", type=Path, help="场地 manifest.json；省略时自动寻找默认场地")
    parser.add_argument("--dart", type=Path, help="可选镖体 manifest.json；网格和估计物性独立读取")
    parser.add_argument("--launch", type=Path, help="可选发射方案 JSON；包含发射点和静止靶平面")
    parser.add_argument("--target-mode", choices=TARGET_MODES, default="fixed", help="靶运动规则模式")
    parser.add_argument("--target-seed", type=int, default=2027, help="移动靶随机种子，用于重放")
    parser.add_argument("--aero-model", choices=("estimated_passive", "none"),
                        default="estimated_passive", help="气动模型：默认估计模型或关闭气动对照")
    parser.add_argument("--flight-control", choices=("on", "off"), default="off",
                        help="开启/关闭Guidedemo飞控，默认关闭")
    parser.add_argument("--flight-config", type=Path, help="可选飞控/舵机/估计舵效配置JSON")
    parser.add_argument("--imu-noise-scale", type=float,
                        help="IMU噪声倍率，0关闭、正数开启；省略时使用飞控配置")
    parser.add_argument("--output", type=Path, help="显式保存结果的目录；省略时只打印，不写文件")
    parser.add_argument("--trace", action="store_true", help="配合 --output 保存详细数值轨迹")
    parser.add_argument("--duration", type=float, default=.6)
    args = parser.parse_args(argv)
    if args.trace and args.output is None:
        parser.error("--trace requires --output")
    if __package__:
        from .simulation_setup import resolve_startup_paths
    else:
        from simulation_setup import resolve_startup_paths
    try:
        inputs = resolve_startup_paths(args.config, args.scene, args.dart, args.launch)
        simulation = DemoSimulation(load_config(inputs.config), inputs.scene, inputs.dart, inputs.launch,
                                    target_mode=args.target_mode, target_seed=args.target_seed,
                                    aero_model=args.aero_model,
                                    flight_control_enabled=args.flight_control == "on",
                                    flight_control_config=args.flight_config)
        if inputs.explicit_values:
            # 用户明确填写的字段优先；其余继续沿用模型/发射方案的已生效值。
            simulation.reset(dict(simulation.cfg, **inputs.explicit_values))
        if args.imu_noise_scale is not None:
            simulation.set_imu_noise(args.imu_noise_scale > 0, args.imu_noise_scale)
    except ValueError as exc:
        parser.error(str(exc))
    try:
        tracker = vision_clock = None
        flight_trace = []

        def record_flight_frame():
            details = sample_flight_vision(simulation, tracker)
            observation = simulation.observation()
            if args.trace:
                flight_trace.append({"t_s": simulation.t, "p_W_m": observation["p_W"].tolist(),
                    "visual": {key: details.get(key) for key in ("detected", "locked", "center_px", "sample_t",
                                                                "aim_point_px", "dart_reference_px")},
                    "flight_control": observation["flight_control"]})

        if simulation.flight_control_enabled:
            if __package__:
                from .openmv_tracker import OpenMVTracker
                from .vision_settings import VisionSettings
                from .playback import SampleClock
                from .flight_sensors import sample_flight_vision
            else:
                from openmv_tracker import OpenMVTracker
                from vision_settings import VisionSettings
                from playback import SampleClock
                from flight_sensors import sample_flight_vision
            vision_path = Path(__file__).resolve().parents[1] / "profiles/vision_desktop.json"
            vision_settings = (VisionSettings(**json.loads(vision_path.read_text(encoding="utf-8")))
                               if vision_path.is_file() else VisionSettings.for_profile("simulation"))
            tracker = OpenMVTracker(vision_settings)
            vision_clock = SampleClock(90.)
            vision_clock.reset(simulation.t)
            record_flight_frame()
        while simulation.t < args.duration and not simulation.done:
            simulation.step()
            if vision_clock is not None and vision_clock.consume(simulation.t):
                record_flight_frame()
        report = simulation.observation()
        report = {key: value.tolist() if isinstance(value,np.ndarray) else value for key,value in report.items()}
        report.update({"engine": "MuJoCo " + mujoco.__version__, "graphics_renderer": simulation.graphics_renderer,
                       "calibration": "UNVALIDATED DEMONSTRATION PARAMETERS", "config": simulation.cfg,
                       "scene": simulation.scene_metadata, "dart": simulation.dart_metadata,
                       "aerodynamics": simulation.aero_metadata,
                       "flight_control_model": simulation.flight_control_metadata,
                       "flight_config": simulation.flight_config,
                       "vision_settings": tracker.settings.as_dict() if tracker is not None else None,
                       "launch": simulation.launch_metadata})
        if args.output is not None:
            args.output.mkdir(parents=True, exist_ok=True)
            (args.output/"result.json").write_text(
                json.dumps(report, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
            if args.trace:
                (args.output/"trace.json").write_text(
                    json.dumps(flight_trace, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=True))
    finally:
        simulation.close()


if __name__ == "__main__":
    main()
