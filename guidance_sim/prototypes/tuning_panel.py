"""可嵌入的飞控调参面板；只在明确应用后重置仿真。"""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

if __package__:
    from .guidedemo_control import load_flight_config, AXIS_OVERRIDE_FIELDS, AXIS_OVERRIDE_KEYS
else:
    from guidedemo_control import load_flight_config, AXIS_OVERRIDE_FIELDS, AXIS_OVERRIDE_KEYS


PROJECT = Path(__file__).resolve().parents[1]
SECTIONS = {
    "controller": "常用控制",
    "servo": "舵机",
    "actuation": "估计舵效",
    "imu_noise": "IMU噪声",
}
FIELD_LABELS = {
    "controller": {
        "autopilot_mode": ("飞控算法", "伪攻角三回路 / PID对照"),
        "pitch_enabled": ("启用俯仰控制", "关闭时俯仰舵量为零"),
        "yaw_enabled": ("启用偏航控制", "关闭时偏航舵量为零"),
        "period_s": ("控制周期", "秒"),
        "png_N": ("比例导航系数 N", "无量纲"),
        "closing_speed_mps": ("固定接近速度", "米/秒"),
        "max_accel_g": ("指令过载上限", "g"),
        "k_rate": ("角速率反馈增益", "秒"),
        "k_alpha": ("伪攻角反馈增益", "无量纲"),
        "k_int": ("过载误差积分增益", "度/(g·秒)"),
        "saturation_deg": ("控制器输出限幅", "度"),
        "filter_k_alpha": ("伪攻角系数 Kα", "与Kθ之比无量纲"),
        "filter_k_theta": ("伪攻角系数 Kθ", "与Kα同量纲"),
        "filter_b1_s": ("伪攻角角速率系数", "秒"),
        "filter_tau_s": ("伪攻角滤波时间常数", "秒"),
        "roll_gain": ("原版滚转角度P增益", "仅 legacy_p 使用；无量纲"),
        "roll_enabled": ("启用滚转稳定", "独立开关；飞控总开关关闭仍全部归零"),
        "roll_mode": ("滚转自驾仪模式", "增强模式在视觉失效时仍稳定滚转"),
        "roll_target_deg": ("滚转目标角", "度；增强模式按最短角误差计算"),
        "roll_angle_gain_per_s": ("滚转角度外环增益", "1/秒；增强模式，初试2"),
        "roll_rate_gain_s": ("滚转角速率内环增益", "秒；增强模式，初试0.1"),
        "roll_rate_limit_deg_s": ("滚转目标角速率上限", "度/秒；增强模式，初试60"),
        "roll_saturation_deg": ("滚转轴输出限幅", "度；空值：原版不限幅，增强继承公共限幅；初试15"),
        "vision_timeout_s": ("视觉保持超时", "秒"),
        "aim_pitch_gain_g_per_deg": ("红色光标俯仰瞄准误差增益", "g/度"),
        "aim_yaw_gain_g_per_deg": ("红色光标偏航瞄准误差增益", "g/度"),
        "aim_accel_limit_g": ("瞄准修正过载限幅", "g"),
    },
    "servo": {
        "limit_deg": ("机械行程限位", "度"),
        "rate_deg_s": ("舵机最大转速", "度/秒"),
        "tau_s": ("舵机响应时间常数", "秒"),
        "pwm_period_s": ("PWM更新周期", "秒"),
        "center_us": ("PWM中位脉宽", "微秒"),
        "us_per_deg": ("PWM角度换算", "微秒/度"),
        "pwm_min_us": ("PWM最小脉宽", "微秒"),
        "pwm_max_us": ("PWM最大脉宽", "微秒"),
    },
    "actuation": {
        "normal_per_rad": ("法向力系数偏舵导数", "1/弧度"),
        "pitch_moment_per_rad": ("俯仰力矩系数偏舵导数", "1/弧度"),
        "yaw_moment_per_rad": ("偏航力矩系数偏舵导数", "1/弧度"),
        "roll_moment_per_rad": ("滚转力矩系数偏舵导数", "1/弧度"),
        "drag_per_rad2": ("附加阻力二次系数", "1/弧度²"),
    },
    "imu_noise": {
        "scale": ("总噪声倍率", "无量纲"),
        "attitude_std_deg": ("姿态角噪声标准差", "度"),
        "rate_std_deg_s": ("角速率噪声标准差", "度/秒"),
        "accel_std_g": ("加速度噪声标准差", "g"),
        "seed": ("随机种子", "非负整数"),
        "period_s": ("噪声采样周期", "秒"),
    },
}

MODE_LABELS = {
    "guidedemo_v1": "伪攻角三回路（v1）",
    "acceleration_pid": "过载 PID（对照）",
}
ROLL_MODE_LABELS = {"legacy_p": "原版角度 P（随视觉门控）",
                    "angle_rate": "角度外环＋角速率内环"}
OPTIONAL_CONTROLLER_KEYS = AXIS_OVERRIDE_KEYS | {"roll_saturation_deg"}
SIGN_LABELS = {
    "los_sign": ("视觉角正负", "作用于图像视线角"),
    "gyro_compensation_sign": ("陀螺补偿正负", "作用于视线角速率补偿"),
    "guidance_sign": ("导引过载正负", "作用于导引生成的过载指令"),
    "accel_feedback_sign": ("实际过载反馈正负", "作用于送入飞控的过载反馈"),
    "rate_feedback_sign": ("角速率反馈正负", "作用于送入飞控的角速率反馈"),
    "output_sign": ("轴舵量输出正负", "作用于混控前的轴舵量"),
}
for _axis, _title in (("pitch", "俯仰"), ("yaw", "偏航")):
    for _field in AXIS_OVERRIDE_FIELDS:
        _label, _unit = FIELD_LABELS["controller"][_field]
        FIELD_LABELS["controller"][f"{_axis}_{_field}"] = (f"{_title}{_label}（覆盖）",
            f"{_unit}；留空继承公共 {_field}")
    for _suffix, (_label, _meaning) in SIGN_LABELS.items():
        FIELD_LABELS["controller"][f"{_axis}_{_suffix}"] = (f"{_title}{_label}", f"±1；{_meaning}")
    for _suffix, _label, _unit in (
        ("kp_deg_per_g", "过载比例 Kp", "度/g"),
        ("ki_deg_per_g_s", "过载积分 Ki", "度/(g·秒)"),
        ("kd_deg_s_per_g", "过载微分 Kd", "度·秒/g"),
        ("integral_limit_g_s", "过载积分状态限幅", "g·秒"),
        ("rate_gain_s", "角速率阻尼增益", "秒"),
    ):
        FIELD_LABELS["controller"][f"pid_{_axis}_{_suffix}"] = (f"{_title}{_label}", _unit)


def _controller_group(key):
    if key.startswith("roll_"):
        return "滚转独立稳定控制"
    if key in AXIS_OVERRIDE_KEYS:
        return "俯仰独立覆盖（留空继承公共）" if key.startswith("pitch_") else "偏航独立覆盖（留空继承公共）"
    if key in ("autopilot_mode", "pitch_enabled", "yaw_enabled", "period_s", "vision_timeout_s"):
        return "运行与控制轴"
    if any(key == f"{axis}_{suffix}" for axis in ("pitch", "yaw") for suffix in SIGN_LABELS):
        return "各环节正负关系（每项独立）"
    if key.startswith("pid_"):
        return "可选过载 PID 系数（仅 PID 模式采用）"
    if key in ("png_N", "closing_speed_mps", "max_accel_g", "aim_pitch_gain_g_per_deg",
               "aim_yaw_gain_g_per_deg", "aim_accel_limit_g"):
        return "导引与红色光标瞄准"
    return "伪攻角三回路系数与公共输出（v1）"


class FlightTuningPanel:
    """编辑副本并通过仿真的原子API应用；不暂停或阻塞主窗口。"""

    def __init__(self, root, simulation, on_applied_callback, *, parent=None):
        self.simulation = simulation
        self.on_applied_callback = on_applied_callback
        self._base_config = copy.deepcopy(simulation.flight_config)
        self.entries: dict[tuple[str, str], tk.StringVar] = {}
        self.boolean_entries: dict[tuple[str, str], tk.BooleanVar] = {}
        self.widgets: dict[tuple[str, str], tk.Widget] = {}
        self._value_kinds: dict[tuple[str, str], type] = {}
        self._canvases: dict[str, tk.Canvas] = {}
        self._embedded = parent is not None
        self._main_layout_mode = None
        if self._embedded:
            self.window = ttk.Frame(parent)
            self.window.pack(fill="both", expand=True)
            width = max(420, parent.winfo_width())
        else:
            self.window = tk.Toplevel(root)
            self.window.title("飞控调参 · 应用后重置本轮")
            width = min(1040, max(650, self.window.winfo_screenwidth()-80))
            height = min(820, max(500, self.window.winfo_screenheight()-100))
            self.window.geometry(f"{width}x{height}")
            self.window.minsize(min(420, width), min(500, height))
            self.window.transient(root)
            self.window.protocol("WM_DELETE_WINDOW", self.close)
        self._dialog_parent = self.window.winfo_toplevel()
        self.noise_enabled = tk.BooleanVar(master=self.window)
        self.boolean_entries[("imu_noise", "enabled")] = self.noise_enabled
        self.status = tk.StringVar(master=self.window, value="编辑为待应用值；应用后从起点重新运行。")
        self.field_help = tk.StringVar(master=self.window,
            value="点击输入框查看字段名和说明；轴覆盖留空即继承公共值。")
        body = ttk.Frame(self.window, padding=(10, 8))
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=1)
        body.rowconfigure(1, weight=1)
        self._heading_label = ttk.Label(body, text="空白轴参数继承公共值；编辑后点击应用，重开本轮。",
                                        wraplength=max(300, width-32))
        self._heading_label.grid(row=0, column=0, sticky="w", pady=(0, 5))
        self.notebook = ttk.Notebook(body)
        self.notebook.grid(row=1, column=0, sticky="nsew")
        for section, title in SECTIONS.items():
            self._build_section(section, title)
        self._help_label = ttk.Label(body, textvariable=self.field_help, wraplength=max(300, width-32))
        self._help_label.grid(row=2, column=0, sticky="w", pady=(5, 3))
        buttons = ttk.Frame(body)
        buttons.grid(row=3, column=0, sticky="w", pady=(2, 4))
        actions = (("应用并重置", self.apply), ("恢复当前", self.restore_current),
                   ("保存JSON", self.save_config), ("加载JSON", self.load_config))
        for index, (text, command) in enumerate(actions):
            ttk.Button(buttons, text=text, width=10, command=command).grid(
                row=0, column=index, sticky="w", padx=(0, 5 if index < 3 else 0))
        self._status_label = ttk.Label(body, textvariable=self.status, wraplength=max(300, width-32))
        self._status_label.grid(row=4, column=0, sticky="w")
        body.bind("<Configure>", lambda event: [label.configure(wraplength=max(280, event.width-4))
            for label in (self._heading_label, self._help_label, self._status_label)], add="+")
        # Bind only our own descendants; embedded frames do not get child events
        # through the toplevel bindtag, and root bindings must remain untouched.
        self._bind_wheel_tree(self.window)
        self._fill(self._base_config)

    def _bind_wheel_tree(self, widget):
        widget.bind("<MouseWheel>", self._mousewheel, add="+")
        for child in widget.winfo_children():
            self._bind_wheel_tree(child)

    def _scroll_page(self, title):
        page = ttk.Frame(self.notebook)
        self.notebook.add(page, text=title)
        page.columnconfigure(0, weight=1)
        page.rowconfigure(0, weight=1)
        canvas = tk.Canvas(page, highlightthickness=0, borderwidth=0)
        scrollbar = ttk.Scrollbar(page, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        fields = ttk.Frame(canvas, padding=(7, 7))
        fields.columnconfigure(0, weight=1)
        item = canvas.create_window((0, 0), window=fields, anchor="nw")
        fields.bind("<Configure>", lambda _event, c=canvas: c.configure(scrollregion=c.bbox("all")))
        canvas.bind("<Configure>", lambda event, c=canvas, i=item:
                    c.itemconfigure(i, width=max(1, event.width)))
        self._canvases[str(page)] = canvas
        return fields

    def _input_widget(self, target, section, key, *, width=9, check_text="开启"):
        identity = (section, key)
        if identity in self.widgets:
            raise ValueError(f"Duplicate tuning field: {section}.{key}")
        value = self._base_config[section][key]
        kind = float if section == "controller" and key in OPTIONAL_CONTROLLER_KEYS else type(value)
        self._value_kinds[identity] = kind
        if kind is bool:
            variable = self.boolean_entries.get(identity)
            if variable is None:
                variable = tk.BooleanVar(master=self.window)
                self.boolean_entries[identity] = variable
            widget = ttk.Checkbutton(target, text=check_text, variable=variable)
        else:
            variable = tk.StringVar(master=self.window)
            self.entries[identity] = variable
            modes = (MODE_LABELS if key == "autopilot_mode" else
                     ROLL_MODE_LABELS if key == "roll_mode" else None)
            if section == "controller" and modes is not None:
                widget = ttk.Combobox(target, textvariable=variable, state="readonly",
                                      values=tuple(modes.values()), width=max(width, 20))
            elif section == "controller" and key.endswith("_sign"):
                widget = ttk.Combobox(target, textvariable=variable, state="readonly",
                                      values=("+1", "-1"), width=6)
            else:
                widget = ttk.Entry(target, textvariable=variable,
                                   state="normal" if kind in (int, float, str) else "readonly",
                                   width=width)
        label, unit = FIELD_LABELS[section].get(key, (key, "按配置定义"))
        if identity == ("imu_noise", "enabled"):
            label, unit = "开启IMU噪声", "按所设倍率和种子采样"
        widget.bind("<FocusIn>", lambda _event, text=f"{section}.{key} · {label} · {unit}":
                    self.field_help.set(text), add="+")
        self.widgets[identity] = widget
        return widget

    def _matrix(self, target, title, rows, headers=("参数（单位）", "公共", "俯仰", "偏航")):
        group = ttk.LabelFrame(target, text=title, padding=(8, 5))
        for column, title in enumerate(headers):
            ttk.Label(group, text=title).grid(row=0, column=column, sticky="w",
                                             padx=(0, 8), pady=(0, 4))
        for row, (label, keys) in enumerate(rows, 1):
            ttk.Label(group, text=label).grid(row=row, column=0, sticky="w",
                                            padx=(0, 10), pady=3)
            for column, key in enumerate(keys, 1):
                if key is None:
                    ttk.Label(group, text="—").grid(row=row, column=column, sticky="w", padx=(0, 8))
                else:
                    self._input_widget(group, "controller", key, width=8).grid(
                        row=row, column=column, sticky="w", padx=(0, 8), pady=3)
        return group

    def _inline_field(self, target, key, label, *, width=8):
        frame = ttk.Frame(target)
        ttk.Label(frame, text=label).pack(side="left", padx=(0, 5))
        self._input_widget(frame, "controller", key, width=width).pack(side="left")
        return frame

    def _build_controller(self, fields):
        self._running = ttk.LabelFrame(fields, text="运行", padding=(8, 5))
        self._running.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 7))
        self._run_mode = self._inline_field(self._running, "autopilot_mode", "算法", width=20)
        self._run_period = self._inline_field(self._running, "period_s", "周期 (s)")
        self._run_timeout = self._inline_field(self._running, "vision_timeout_s", "视觉超时 (s)")
        self._run_switches = ttk.Frame(self._running)
        for key, label in (("pitch_enabled", "俯仰"), ("yaw_enabled", "偏航"), ("roll_enabled", "滚转")):
            self._input_widget(self._run_switches, "controller", key, check_text=label).pack(side="left", padx=(0, 10))

        self._main_left = ttk.Frame(fields)
        controls = self._matrix(self._main_left, "PNG与v1自驾仪 · 空白轴覆盖继承公共值", [
            ("比例 N (1)", ("png_N", "pitch_png_N", "yaw_png_N")),
            ("速率 Kr (s)", ("k_rate", "pitch_k_rate", "yaw_k_rate")),
            ("伪攻角 Ka (1)", ("k_alpha", "pitch_k_alpha", "yaw_k_alpha")),
            ("积分 Ki (°/g/s)", ("k_int", "pitch_k_int", "yaw_k_int")),
            ("接近速度 (m/s)", ("closing_speed_mps", None, None)),
            ("请求上限 (g)", ("max_accel_g", None, None)),
            ("轴输出上限 (°)", ("saturation_deg", None, None))])
        controls.pack(anchor="nw")
        ttk.Label(self._main_left, text="滤波、PID、符号和红十字修正见“高级”。",
                  wraplength=350).pack(anchor="w", pady=(5, 0))

        self._main_roll = ttk.LabelFrame(fields, text="滚转稳定", padding=(8, 5))
        ttk.Label(self._main_roll, text="模式").grid(row=0, column=0, sticky="w", pady=3)
        self._input_widget(self._main_roll, "controller", "roll_mode", width=20).grid(
            row=0, column=1, columnspan=3, sticky="w", pady=3)
        for row, pairs in enumerate((
            (("roll_target_deg", "目标 (°)"), ("roll_gain", "原P增益 (1)")),
            (("roll_angle_gain_per_s", "外环 (1/s)"), ("roll_rate_gain_s", "内环 (s)")),
            (("roll_rate_limit_deg_s", "限速 (°/s)"), ("roll_saturation_deg", "舵量上限 (°)")),
        ), 1):
            for pair, (key, label) in enumerate(pairs):
                ttk.Label(self._main_roll, text=label).grid(row=row, column=pair*2,
                    sticky="w", padx=(0, 5), pady=4)
                self._input_widget(self._main_roll, "controller", key, width=7).grid(
                    row=row, column=pair*2+1, sticky="w", padx=(0, 8), pady=4)
        ttk.Label(self._main_roll,
            text="舵量限幅留空：原P不限幅；增强模式继承公共上限。\n"
                 "增强初试：外环2、内环0.1、限速60、舵量15°。\n"
                 "增强模式在视觉失效时仍运行；总飞控关闭时归零。\n"
                 "增益及舵效仍需实物标定。",
            wraplength=330).grid(row=4, column=0, columnspan=4, sticky="w", pady=(7, 2))
        self._main_fields = fields
        fields.bind("<Configure>", lambda event: self._layout_main(event.width), add="+")
        self._layout_main(600)

    def _layout_main(self, width):
        mode = (width >= 820, width >= 620)
        if mode == self._main_layout_mode:
            return
        self._main_layout_mode = mode
        wide, running_wide = mode
        self._main_fields.columnconfigure(0, weight=0)
        self._main_fields.columnconfigure(1, weight=1 if wide else 0)
        self._main_left.grid(row=1, column=0, sticky="nw", padx=(0, 8 if wide else 0), pady=(0, 7))
        self._main_roll.grid(row=1 if wide else 2, column=1 if wide else 0, sticky="nw", pady=(0, 7))
        self._run_mode.grid(row=0, column=0, columnspan=2, sticky="w", padx=(0, 12), pady=3)
        self._run_switches.grid(row=1 if running_wide else 2, column=0, columnspan=2, sticky="w", pady=3)
        self._run_period.grid(row=0 if running_wide else 1, column=2 if running_wide else 0,
                              sticky="w", padx=(0, 10), pady=3)
        self._run_timeout.grid(row=1, column=2 if running_wide else 1, sticky="w", pady=3)

    def _build_advanced(self):
        fields = self._scroll_page("高级")
        ttk.Label(fields, text="高级项不改变默认算法；v1抗饱和方向已修正。图像视线/过载以下、右为正。",
                  wraplength=355).grid(row=0, column=0, sticky="w", pady=(0, 7))
        specs = (
            ("v1伪攻角滤波 · 空白轴覆盖继承公共", [
                ("Kα (1)", ("filter_k_alpha", "pitch_filter_k_alpha", "yaw_filter_k_alpha")),
                ("Kθ (1)", ("filter_k_theta", "pitch_filter_k_theta", "yaw_filter_k_theta")),
                ("b1 (s)", ("filter_b1_s", "pitch_filter_b1_s", "yaw_filter_b1_s")),
                ("τ (s)", ("filter_tau_s", "pitch_filter_tau_s", "yaw_filter_tau_s"))], None),
            ("过载PID · 仅PID模式使用", [
                ("Kp (°/g)", ("pid_pitch_kp_deg_per_g", "pid_yaw_kp_deg_per_g")),
                ("Ki (°/g/s)", ("pid_pitch_ki_deg_per_g_s", "pid_yaw_ki_deg_per_g_s")),
                ("Kd (°·s/g)", ("pid_pitch_kd_deg_s_per_g", "pid_yaw_kd_deg_s_per_g")),
                ("积分限幅 (g·s)", ("pid_pitch_integral_limit_g_s", "pid_yaw_integral_limit_g_s")),
                ("速率阻尼 (s)", ("pid_pitch_rate_gain_s", "pid_yaw_rate_gain_s"))],
                ("参数（单位）", "俯仰", "偏航")),
            ("红十字瞄准修正", [
                ("误差增益 (g/°)", (None, "aim_pitch_gain_g_per_deg", "aim_yaw_gain_g_per_deg")),
                ("修正上限 (g)", ("aim_accel_limit_g", None, None))], None),
            ("坐标与接线符号 · ±1", [
                (label, ("pitch_"+suffix, "yaw_"+suffix))
                for suffix, (label, _meaning) in SIGN_LABELS.items()],
                ("环节", "俯仰", "偏航")),
        )
        for row, (title, rows, headers) in enumerate(specs, 1):
            group = self._matrix(fields, title, rows, **({"headers": headers} if headers else {}))
            group.grid(row=row, column=0, sticky="nw", pady=(0, 8))
        remaining = [key for key in self._base_config["controller"] if ("controller", key) not in self.widgets]
        if remaining:
            extra = ttk.LabelFrame(fields, text="其他配置", padding=8)
            extra.grid(row=len(specs)+1, column=0, sticky="nw")
            for row, key in enumerate(remaining):
                self._build_field(extra, "controller", key, row)

    def _build_section(self, section, title):
        fields = self._scroll_page(title)
        if section == "controller":
            self._build_controller(fields)
            self._build_advanced()
            return
        group = ttk.LabelFrame(fields, text=title, padding=(8, 5))
        group.grid(row=0, column=0, sticky="nw")
        for row, key in enumerate(self._base_config[section]):
            self._build_field(group, section, key, row)
        if section == "actuation":
            ttk.Label(fields, text="聚合舵效为未标定估计；方向请在高级页调整坐标/接线符号。",
                      wraplength=355).grid(row=1, column=0, sticky="w", pady=7)

    def _build_field(self, target, section, key, row):
        label, unit = FIELD_LABELS[section].get(key, (key, "按配置定义"))
        if (section, key) == ("imu_noise", "enabled"):
            label, unit = "IMU噪声", "开关"
        # Units sit on the same row; technical keys live in focus help, not in
        # a second line under every label.
        short_unit = unit.split("；", 1)[0]
        ttk.Label(target, text=f"{label} ({short_unit})", wraplength=250).grid(
            row=row, column=0, sticky="w", padx=(0, 12), pady=5)
        self._input_widget(target, section, key, width=10).grid(
            row=row, column=1, sticky="w", pady=5)

    def _mousewheel(self, event):
        canvas = self._canvases.get(self.notebook.select())
        if canvas is not None and event.delta:
            canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")
        return "break"

    def _fill(self, config):
        self._base_config = copy.deepcopy(config)
        for (section, key), variable in self.entries.items():
            value = config[section][key]
            if (section, key) == ("controller", "autopilot_mode"):
                variable.set(MODE_LABELS.get(value, value))
            elif (section, key) == ("controller", "roll_mode"):
                variable.set(ROLL_MODE_LABELS.get(value, value))
            elif section == "controller" and key.endswith("_sign"):
                variable.set("+1" if value == 1 else str(value))
            else:
                variable.set("" if value is None else str(value))
        for (section, key), variable in self.boolean_entries.items():
            variable.set(config[section][key])

    def _candidate(self):
        candidate = copy.deepcopy(self._base_config)
        for (section, key), variable in self.entries.items():
            label = FIELD_LABELS[section].get(key, (key, ""))[0]
            kind = self._value_kinds[(section, key)]
            if kind not in (int, float, str):
                continue
            if section == "controller" and key in OPTIONAL_CONTROLLER_KEYS and variable.get().strip() in ("", "继承", "null", "None"):
                candidate[section][key] = None
                continue
            if kind is str:
                value = variable.get()
                if (section, key) == ("controller", "autopilot_mode"):
                    value = {text: name for name, text in MODE_LABELS.items()}.get(value, value)
                elif (section, key) == ("controller", "roll_mode"):
                    value = {text: name for name, text in ROLL_MODE_LABELS.items()}.get(value, value)
                candidate[section][key] = value
                continue
            try:
                value = int(variable.get()) if kind is int else float(variable.get())
            except (ValueError, OverflowError) as exc:
                raise ValueError(f"{label}必须是{'整数' if kind is int else '数字'}。") from exc
            if kind is float and not math.isfinite(value):
                raise ValueError(f"{label}必须是有限数字。")
            candidate[section][key] = value
        for (section, key), variable in self.boolean_entries.items():
            candidate[section][key] = bool(variable.get())
        return candidate

    def apply(self):
        try:
            candidate = self._candidate()
            self.simulation.apply_flight_config(candidate)
        except (ValueError, TypeError, OSError) as exc:
            messagebox.showerror("飞控参数未应用", str(exc), parent=self._dialog_parent)
            return
        self._fill(self.simulation.flight_config)
        self.on_applied_callback()
        self.status.set("已应用并重置本轮；参数文件只有在另行保存后才会写入。")

    def restore_current(self):
        self._fill(self.simulation.flight_config)
        self.status.set("已恢复当前仿真实际采用的参数；本轮运行未改变。")

    def save_config(self):
        path = filedialog.asksaveasfilename(parent=self._dialog_parent, title="保存飞控配置副本",
            initialdir=str(PROJECT/"profiles"), initialfile="flight_control_tuned.json",
            defaultextension=".json", filetypes=(("JSON配置", "*.json"),), confirmoverwrite=True)
        if not path:
            return
        try:
            candidate = load_flight_config(self._candidate())
            Path(path).write_text(json.dumps(candidate, ensure_ascii=False, indent=2, allow_nan=False)+"\n",
                                  encoding="utf-8")
        except (ValueError, TypeError, OSError) as exc:
            messagebox.showerror("飞控配置未保存", str(exc), parent=self._dialog_parent)
            return
        self.status.set(f"已保存配置副本：{path}；当前仿真未改变。")

    def load_config(self):
        path = filedialog.askopenfilename(parent=self._dialog_parent, title="加载飞控JSON到表单",
            initialdir=str(PROJECT/"profiles"), filetypes=(("JSON配置", "*.json"),))
        if not path:
            return
        try:
            candidate = load_flight_config(path)
        except (ValueError, TypeError, OSError) as exc:
            messagebox.showerror("飞控配置未加载", str(exc), parent=self._dialog_parent)
            return
        self._fill(candidate)
        self.status.set(f"已加载待应用参数：{path}；点击“应用并重置”后生效。")

    def close(self):
        try:
            self.window.destroy()
        except tk.TclError:
            pass
