"""MuJoCo控制台：第三视角与竖排曲线，独立镖载相机窗口。

物理运动由 dart_demo.py 的 DemoSimulation 计算，这里不重写动力学。
画面、诊断与回放共用一个仿真，显示操作不推进控制或传感器。
可通过工作区启动文件直接运行；仅导入本模块不会创建窗口。
"""

from __future__ import annotations

import argparse
import copy
from dataclasses import replace
from datetime import datetime
import json
import math
from pathlib import Path
import time
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Any

from PIL import Image, ImageTk

try:
    from .openmv_tracker import OpenMVTracker
    from .playback import ReplayBuffer, SampleClock
    from .field_scene import available_scenes
    from .vision_settings import VisionSettings
    from .target_motion import TARGET_MODES
    from .tuning_panel import FlightTuningPanel, ROLL_MODE_LABELS
    from .flight_debug import DiagnosticBuffer, FlightDebugWindow, render_g_hud
except ImportError:
    from openmv_tracker import OpenMVTracker
    from playback import ReplayBuffer, SampleClock
    from field_scene import available_scenes
    from vision_settings import VisionSettings
    from target_motion import TARGET_MODES
    from tuning_panel import FlightTuningPanel, ROLL_MODE_LABELS
    from flight_debug import DiagnosticBuffer, FlightDebugWindow, render_g_hud


ROOT = Path(__file__).resolve().parents[1]
COLORS = {
    "background": "#EDF3F8",
    "surface": "#FFFFFF",
    "text": "#23364A",
    "muted": "#556C7F",
    "accent": "#176B9A",
    "viewport": "#182B3C",
    "notice": "#FFF1CD",
}

# 这些范围只用于拦截明显不合理的教学输入，不代表实物硬件的工作范围。
PARAMETERS = (
    ("speed_mps", "初始速度", "米 / 秒", 0.0, 100.0),
    ("pitch_deg", "初始仰角", "度", -89.0, 89.0),
    ("yaw_deg", "初始偏航", "度", -180.0, 180.0),
    ("mass_kg", "质量", "千克", 0.001, 20.0),
    ("wind_cross_mps", "横风", "米 / 秒", -30.0, 30.0),
    ("drag_coefficient", "阻力系数", "无单位", 0.0, 10.0),
    ("guide_light_brightness", "绿灯显示亮度", "0 到 1", 0.0, 1.0),
)

STATUS_TEXT = {
    "paused": "位于起点",
    "running": "飞行中",
    "landed": "已触地",
    "finished": "已到演示时长",
    "invalid": "数值异常",
    "target_hit": "几何命中目标",
    "target_miss": "过靶但未命中",
}


class DartViewer:
    """连接用户界面和仿真后端，区分真实时间、物理步长和画面刷新。"""

    def __init__(self, root: tk.Tk, simulation: Any, *, auto_run: bool = False, vision_settings=None):
        """接收同一个仿真，建立单窗口控制台并绘制初始双画面。"""
        self.root = root
        self.sim = simulation
        self.running = False
        self.closed = False
        self.accumulator = 0.0  # 已经过、但尚未用于物理计算的真实秒数。
        self.last_wall = time.perf_counter()  # 单调计时器，不受系统时间校准影响。
        self.last_render = 0.0
        self.dirty = True  # 状态或窗口改变时置 True，表示画面需要重新生成。
        self._after_id: str | None = None
        self._photos: dict[str, ImageTk.PhotoImage] = {}
        self._external_resize_after = None
        self._external_pending_size = None
        self._external_resize_deadline = 0.
        self._replay_frame_index = None
        self._image_rects = {}
        self._flight_tuning_panel = None
        self._axis_tuning_hints = {}
        self._axis_tuning_signatures = {}
        self.diagnostics = DiagnosticBuffer(max_samples=10000)
        self._flight_debug_window = None
        self.tracker = getattr(self, "tracker_factory", OpenMVTracker)(
            vision_settings or VisionSettings.for_profile("simulation"))
        self._vision_image = None
        self._vision_snapshot = {}
        # 最近一次视觉处理对应的仿真时刻；保存截图不能偷偷多处理一帧。
        self._vision_t: float | None = None
        self._vision_raw = None
        self._vision_source = None
        self.vision_clock = SampleClock(90.)
        self.history = ReplayBuffer()
        self.replaying = False
        self.replay_cursor = 0.
        self._live_state = None
        self._live_vision = None
        self._updating_timeline = False
        self.vision_count = self.vision_detected = self.vision_missed = 0
        self.vision_work_s = self.active_wall_s = 0.
        self._drag_origin: tuple[int, int, float, float] | None = None
        self.entries: dict[str, tk.StringVar] = {}
        self.message = tk.StringVar(value="已暂停。点击“开始”重放；修改参数后需要重新检查落点。")
        self.metrics = tk.StringVar()
        self.flight_telemetry = tk.StringVar()
        self.onboard_clock = tk.StringVar()
        self.vision_status = tk.StringVar(value="视觉识别等待画面。")
        self.speed_choice = tk.StringVar(value="1×")
        self.image_choice = tk.StringVar(value="识别叠加")
        self.timeline_text = tk.StringVar(value="尚未录制")
        self.vision_profile = tk.StringVar(value={"openmv": "原OpenMV", "simulation": "仿真绿灯"}.get(
            self.tracker.settings.profile, "自定义"))
        self.lab_entries = [tk.StringVar(value=str(value)) for value in self.tracker.settings.lab]
        self.exposure_entry = tk.StringVar(value=str(self.tracker.settings.exposure_us))
        self.bloom_radius_entry = tk.StringVar(value=str(self.tracker.settings.bloom_radius_px))
        self.bloom_gain_entry = tk.StringVar(value=str(self.tracker.settings.bloom_gain))
        self.aim_point_entries = [tk.StringVar(value=str(v)) for v in self.tracker.settings.aim_point_px]
        self.dart_reference_entries = [tk.StringVar(value=str(v)) for v in self.tracker.settings.dart_reference_px]
        self.aim_reference_mode = tk.StringVar(value={"image_reference": "参与靶面微调", "display_only": "仅显示参考"}.get(
            (getattr(self.sim, "_aim_reference_config", None) or {}).get("mode", "display_only"), "仅显示参考"))
        self.cursor_pick_mode = tk.StringVar(value="不点选")
        self.state_label = tk.StringVar(value="暂停中")
        self.target_choice = tk.StringVar(value=TARGET_MODES[simulation.target_mode])
        self.target_seed = tk.StringVar(value=str(simulation.target_seed))
        self.flight_control_choice = tk.StringVar(
            value="开启飞控" if simulation.flight_control_enabled else "关闭飞控")
        noise = simulation.flight_config["imu_noise"]
        self.imu_noise_enabled = tk.BooleanVar(value=noise["enabled"])
        self.imu_noise_scale = tk.StringVar(value=f"{noise['scale']:g}")
        self._build()
        self._fill_parameters()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<space>", self._space_key)
        self.root.bind("<Escape>", lambda _event: self.close())
        self.root.update_idletasks()
        self._sample_vision()
        self._record_flight_debug()
        self._render()
        # 初次窗口布局/贴图先完成，再给自动运行建立现实计时起点。
        self.root.update_idletasks()
        if auto_run:
            self.toggle_running()
        # 约 2 毫秒检查一次；界面显示仍为 30 Hz，视觉按仿真时间独立采样。
        self._after_id = self.root.after(2, self._tick)

    def _build(self) -> None:
        """主窗口显示第三视角与竖排曲线，相机端独立显示。"""
        self.root.title("飞镖控制台 · 三轴飞控与视觉仿真")
        screen_width, screen_height = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        width = min(1580, max(900, screen_width - 48))
        height = min(950, max(650, screen_height - 100))
        self.root.geometry(f"{width}x{height}+18+28")
        self.root.minsize(min(960, width), min(650, height))
        self.root.configure(background=COLORS["background"])
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("TFrame", background=COLORS["background"])
        style.configure("TLabelframe", background=COLORS["background"], bordercolor="#CAD9E4")
        style.configure("TLabelframe.Label", background=COLORS["background"],
                        foreground=COLORS["text"], font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("TLabel", background=COLORS["background"], foreground=COLORS["text"],
                        font=("Microsoft YaHei UI", 10))
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 17, "bold"))
        style.configure("Section.TLabel", font=("Microsoft YaHei UI", 11, "bold"))
        style.configure("Muted.TLabel", foreground=COLORS["muted"])
        style.configure("TButton", font=("Microsoft YaHei UI", 10), padding=(10, 5),
                        background=COLORS["surface"], foreground=COLORS["text"],
                        borderwidth=1, bordercolor="#CAD9E4", lightcolor=COLORS["surface"],
                        darkcolor="#CAD9E4", relief="flat")
        style.map("TButton", background=[("pressed", "#D6E8F4"), ("active", "#E4F0F7")],
                  foreground=[("disabled", COLORS["muted"])],
                  bordercolor=[("focus", COLORS["accent"])])
        style.configure("Accent.TButton", background=COLORS["accent"], foreground="white",
                        bordercolor=COLORS["accent"], lightcolor=COLORS["accent"],
                        darkcolor=COLORS["accent"])
        style.configure("Small.TButton", padding=(4, 2), font=("Microsoft YaHei UI", 9))
        style.map("Accent.TButton", background=[("pressed", "#104C6D"), ("active", "#12577E")],
                  foreground=[("disabled", "#E0EAF0"), ("!disabled", "white")])
        style.configure("TEntry", font=("Microsoft YaHei UI", 10), padding=4,
                        fieldbackground=COLORS["surface"], bordercolor="#CAD9E4")
        style.configure("TCombobox", fieldbackground=COLORS["surface"], background=COLORS["surface"],
                        foreground=COLORS["text"], bordercolor="#CAD9E4", arrowsize=12)
        style.map("TCombobox", fieldbackground=[("readonly", COLORS["surface"])],
                  selectbackground=[("readonly", COLORS["surface"])],
                  selectforeground=[("readonly", COLORS["text"])])
        style.configure("TCheckbutton", background=COLORS["background"], foreground=COLORS["text"])
        style.map("TCheckbutton", background=[("active", COLORS["background"])])
        style.configure("TNotebook", background=COLORS["background"], borderwidth=0)
        style.configure("TNotebook.Tab", padding=(16, 6), font=("Microsoft YaHei UI", 10),
                        background="#E1ECF4", foreground=COLORS["muted"], borderwidth=1,
                        lightcolor="#CAD9E4", darkcolor="#CAD9E4")
        style.map("TNotebook.Tab", background=[("selected", COLORS["surface"]), ("active", "#E4F0F7")],
                  foreground=[("selected", COLORS["accent"]), ("active", COLORS["text"])])
        style.configure("TPanedwindow", background=COLORS["background"])

        outer = ttk.Frame(self.root, padding=(12, 8, 12, 8))
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(2, weight=1)
        heading = ttk.Frame(outer)
        heading.grid(row=0, column=0, sticky="ew", pady=(0, 7))
        ttk.Label(heading, text="飞镖控制台", style="Title.TLabel").pack(side="left")
        self.run_summary = tk.StringVar(value="准备开始")
        ttk.Label(heading, textvariable=self.run_summary, foreground=COLORS["accent"],
                  font=("Microsoft YaHei UI", 10)).pack(side="left", padx=18)
        ttk.Label(heading, textvariable=self.state_label, foreground=COLORS["accent"],
                  font=("Microsoft YaHei UI", 10, "bold")).pack(side="right")

        toolbar = ttk.Frame(outer)
        toolbar.grid(row=1, column=0, sticky="ew", pady=(0, 6))
        self.scene_paths = available_scenes()
        current_scene = self.sim.scene_metadata.get("manifest")
        selected = next((name for name, path in self.scene_paths.items()
                         if current_scene and path == Path(current_scene)), "自定义场地")
        self.scene_choice = tk.StringVar(value=selected)
        ttk.Label(toolbar, text="场地").pack(side="left")
        scene_selector = ttk.Combobox(toolbar, textvariable=self.scene_choice,
                                     values=tuple(self.scene_paths), state="readonly", width=10)
        scene_selector.pack(side="left", padx=(5, 14))
        scene_selector.bind("<<ComboboxSelected>>", self._change_scene)
        self.flight_control_selector = ttk.Combobox(toolbar, textvariable=self.flight_control_choice,
            values=("关闭飞控", "开启飞控"), state="readonly", width=10)
        self.flight_control_selector.pack(side="left")
        self.flight_control_selector.bind("<<ComboboxSelected>>", self._change_flight_control)
        ttk.Button(toolbar, text="导引 / 飞控调参", command=self._open_flight_tuning).pack(
            side="left", padx=(8, 5))
        ttk.Button(toolbar, text="视觉设置", command=lambda: self._show_settings("vision")).pack(side="left")
        ttk.Button(toolbar, text="发射与环境", command=lambda: self._show_settings("launch")).pack(
            side="left", padx=5)
        ttk.Button(toolbar, text="飞控曲线", command=self._open_flight_debug).pack(side="left")
        ttk.Button(toolbar, text="状态详情", command=lambda: self._show_settings("details")).pack(
            side="right")

        self.workspace_tabs = ttk.Notebook(outer)
        self.workspace_tabs.grid(row=2, column=0, sticky="nsew")
        self.flight_page = ttk.Frame(self.workspace_tabs)
        self.settings_page = ttk.Frame(self.workspace_tabs)
        self.workspace_tabs.add(self.flight_page, text="飞行与曲线")
        self.workspace_tabs.add(self.settings_page, text="设置与详情")
        self.flight_page.rowconfigure(0, weight=1)
        self.flight_page.columnconfigure(0, weight=1)

        self.flight_horizontal = ttk.Panedwindow(self.flight_page, orient="horizontal")
        self.flight_horizontal.grid(row=0, column=0, sticky="nsew", pady=(5, 0))
        self.external_host = ttk.Frame(self.flight_horizontal, padding=(0, 0, 7, 0))
        self.external_host.columnconfigure(0, weight=1)
        self.external_host.rowconfigure(1, weight=1)
        self.flight_horizontal.add(self.external_host, weight=7)
        self.external_header = ttk.Frame(self.external_host)
        self.external_header.grid(row=0, column=0, sticky="ew", pady=(0, 5))
        ttk.Label(self.external_header, text="第三视角", style="Section.TLabel").pack(side="left")
        view_names = {"overview": "场地全景", "follow": "跟随镖体", "base": "基地后方看发射架"}
        self.view_choice = tk.StringVar(value=view_names.get(self.sim.external_view, "跟随镖体"))
        view_options = ("场地全景", "跟随镖体", "基地后方看发射架") if self.sim.launch_profile else (
            "场地全景", "跟随镖体")
        self.view_selector = ttk.Combobox(self.external_header, textvariable=self.view_choice,
                                        values=view_options, state="readonly", width=16)
        self.view_selector.pack(side="right")
        self.view_selector.bind("<<ComboboxSelected>>", self._change_external_view)
        ttk.Button(self.external_header, text="复位", command=self._reset_external_camera).pack(
            side="right", padx=5)
        self.g_hud_target = tk.StringVar(value="第三视角")
        self.g_hud_selector = ttk.Combobox(self.external_header, textvariable=self.g_hud_target,
            values=("第三视角", "镖载视觉", "关闭"), state="readonly", width=10)
        self.g_hud_selector.pack(side="right", padx=(4, 8))
        self.g_hud_selector.bind("<<ComboboxSelected>>", lambda _event: self._request_render())
        ttk.Label(self.external_header, text="g叠加", style="Muted.TLabel").pack(side="right")
        self.external_label = tk.Label(self.external_host, bg=COLORS["viewport"], bd=0)
        self.external_label.grid(row=1, column=0, sticky="nsew")
        self.external_label.bind("<Configure>", lambda _event: self._request_render())
        self.external_label.bind("<ButtonPress-1>", self._camera_drag_start)
        self.external_label.bind("<B1-Motion>", self._camera_drag_move)
        self.external_label.bind("<MouseWheel>", self._camera_wheel)
        self.external_label.bind("<Button-4>", lambda _event: self._camera_zoom(1))
        self.external_label.bind("<Button-5>", lambda _event: self._camera_zoom(-1))
        self.scene_label = ttk.Label(self.external_host, text=self.sim.scene_label, style="Muted.TLabel",
                                    font=("Microsoft YaHei UI", 8), wraplength=850)
        self.scene_label.grid(row=2, column=0, sticky="w", pady=(3, 0))
        self.external_host.bind("<Configure>", lambda event: self.scene_label.configure(
            wraplength=max(200, event.width-12)), add="+")
        self.debug_host = ttk.Frame(self.flight_horizontal, width=460)
        self.flight_horizontal.add(self.debug_host, weight=3)
        self._flight_debug_window = FlightDebugWindow(
            self.root, self.diagnostics, lambda: float(self.sim.t),
            parent=self.debug_host, stacked=True,
            on_tune_axis=None if hasattr(self.sim, "expert_config") else self._toggle_axis_tuning)
        self._flight_debug_window.show()
        self._build_onboard_window()

        self.settings_tabs = ttk.Notebook(self.settings_page)
        self.settings_tabs.pack(fill="both", expand=True, pady=(6, 0))
        self._settings_pages = {}
        for section, title in (("launch", "发射与环境"),
                               ("flight", "导引与飞控"), ("details", "状态详情")):
            tab = ttk.Frame(self.settings_tabs)
            self.settings_tabs.add(tab, text=title)
            self._settings_pages[section] = tab
            if section == "flight":
                host = ttk.Frame(tab, padding=(12, 8))
                host.pack(fill="both", expand=True)
            else:
                host = self._scrolling_settings(tab)
            setattr(self, {"launch": "launch_settings_host", "vision": "vision_settings_host",
                           "flight": "flight_tuning_host", "details": "details_host"}[section], host)
        self._build_launch_settings(self.launch_settings_host)
        ttk.Label(self.flight_tuning_host, text="点击上方“导引 / 飞控调参”打开当前算法的参数。",
                  style="Muted.TLabel").pack(anchor="w", pady=8)
        self._build_details(self.details_host)
        self.settings_tabs.bind("<<NotebookTabChanged>>", self._settings_tab_changed)

        controls = ttk.Frame(outer)
        controls.grid(row=3, column=0, sticky="ew", pady=(7, 4))
        self.play_button = ttk.Button(controls, text="开始", command=self.toggle_running,
                                      style="Accent.TButton")
        self.play_button.pack(side="left")
        ttk.Button(controls, text="重置并应用参数", command=self.reset).pack(side="left", padx=7)
        ttk.Button(controls, text="保存画面与配置", command=self.save_capture).pack(side="left")
        ttk.Button(controls, text="显示第一视角", command=self.show_onboard).pack(side="left", padx=7)
        ttk.Label(controls, text="拖动分隔线调整侧栏宽度；相机独立显示", style="Muted.TLabel",
                  font=("Microsoft YaHei UI", 9)).pack(side="right")

        playback = ttk.Frame(outer)
        playback.grid(row=4, column=0, sticky="ew", pady=(0, 3))
        playback.columnconfigure(1, weight=1)
        ttk.Button(playback, text="从头回放", command=self.start_replay).grid(row=0, column=0, padx=(0, 7))
        self.timeline = ttk.Scale(playback, from_=0., to=.001, command=self._seek_replay)
        self.timeline.grid(row=0, column=1, sticky="ew")
        ttk.Button(playback, text="回到最新", command=self.return_live).grid(row=0, column=2, padx=7)
        speed = ttk.Combobox(playback, textvariable=self.speed_choice,
                             values=("0.1×", "0.25×", "0.5×", "1×", "2×"), state="readonly", width=7)
        speed.grid(row=0, column=3)
        speed.bind("<<ComboboxSelected>>", self._change_speed)
        ttk.Label(playback, textvariable=self.timeline_text, style="Muted.TLabel",
                  font=("Microsoft YaHei UI", 9)).grid(row=1, column=0, columnspan=4, sticky="w", pady=(2, 0))
        self.message_label = ttk.Label(outer, textvariable=self.message, style="Muted.TLabel",
                                      wraplength=width-35, font=("Microsoft YaHei UI", 9), justify="left")
        self.message_label.grid(row=5, column=0, sticky="w")
        outer.bind("<Configure>", lambda event: self.message_label.configure(
            wraplength=max(400, event.width-20)))
        self._initial_panes_positioned = False
        self._initial_panes_user_adjusted = False
        self.flight_horizontal.bind("<ButtonPress-1>", self._mark_initial_pane_drag, add="+")
        self.flight_horizontal.bind("<Map>", lambda _event: self.root.after(
            80, self._initial_pane_positions), add="+")
        # after_idle can precede geometry negotiation through nested Notebooks.
        # A short post-Map delay uses the actual available flight-page height.
        self.root.after(80, self._initial_pane_positions)

    def _mark_initial_pane_drag(self, _event=None) -> None:
        self._initial_panes_user_adjusted = True

    def _initial_pane_positions(self) -> None:
        """只在布局稳定后设置侧栏宽度；用户调整后不再覆盖。"""
        if (self.closed or self._initial_panes_positioned
                or self._initial_panes_user_adjusted or not self.flight_horizontal.winfo_ismapped()):
            return
        available = self.flight_horizontal.winfo_width()
        if available < 200:
            return
        sidebar_width = min(500, max(430, round(available * .29)))
        self.flight_horizontal.sashpos(0, max(320, available-sidebar_width-6))
        self._initial_panes_positioned = True

    def _scrolling_settings(self, parent) -> ttk.Frame:
        """设置页使用自身滚动区，长参数不再挤压飞行画面。"""
        parent.rowconfigure(0, weight=1)
        parent.columnconfigure(0, weight=1)
        canvas = tk.Canvas(parent, bg=COLORS["background"], highlightthickness=0, width=1, height=1)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        body = ttk.Frame(canvas, padding=(16, 12))
        window = canvas.create_window(0, 0, window=body, anchor="nw")
        body.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window, width=event.width))
        def wheel(event):
            canvas.yview_scroll(-1 if event.delta > 0 or getattr(event, "num", 0) == 4 else 1, "units")
            return "break"
        canvas.bind("<MouseWheel>", wheel)
        canvas.bind("<Button-4>", wheel)
        canvas.bind("<Button-5>", wheel)
        # Events over child entries/buttons normally never reach their Canvas parent.
        # Bind the same local scroller to descendants after their controls are built.
        def bind_children():
            def walk(widget):
                for child in widget.winfo_children():
                    if not getattr(child, "_viewer_settings_scroller_bound", False):
                        child.bind("<MouseWheel>", wheel, add="+")
                        child.bind("<Button-4>", wheel, add="+")
                        child.bind("<Button-5>", wheel, add="+")
                        child._viewer_settings_scroller_bound = True
                    walk(child)
            if body.winfo_exists():
                walk(body)
        body.bind("<Map>", lambda _event: bind_children(), add="+")
        return body

    def _show_settings(self, section="flight") -> None:
        if section == "vision":
            self.show_onboard()
            self.onboard_tabs.select(self.onboard_settings_page)
            return
        self.workspace_tabs.select(self.settings_page)
        self.settings_tabs.select(self._settings_pages[section])

    def _settings_tab_changed(self, _event=None) -> None:
        if (self.settings_tabs.select() == str(self._settings_pages["flight"])
                and self._flight_tuning_panel is None):
            self._open_flight_tuning()

    def _build_launch_settings(self, host) -> None:
        ttk.Label(host, text="发射与环境", style="Section.TLabel").pack(anchor="w")
        ttk.Label(host, text="编辑后点击“重置并应用参数”重新开始本轮。",
                  style="Muted.TLabel").pack(anchor="w", pady=(4, 12))
        fields = ttk.Frame(host)
        fields.pack(fill="x", pady=(0, 12))
        fields.columnconfigure(1, weight=1)
        fields.columnconfigure(4, weight=1)
        for index, (key, title, unit, _low, _high) in enumerate(PARAMETERS):
            row, group = divmod(index, 2)
            column = group * 3
            ttk.Label(fields, text=title).grid(row=row, column=column, sticky="w", padx=(0, 8), pady=6)
            value = tk.StringVar()
            self.entries[key] = value
            ttk.Entry(fields, textvariable=value, width=14).grid(
                row=row, column=column+1, sticky="ew", padx=(0, 8), pady=6)
            ttk.Label(fields, text=unit, style="Muted.TLabel").grid(
                row=row, column=column+2, sticky="w", padx=(0, 30), pady=6)
        ttk.Button(host, text="重置并应用参数", command=self.reset, style="Accent.TButton").pack(
            anchor="w", pady=(0, 14))
        targets = ttk.LabelFrame(host, text="基地目标", padding=10)
        targets.pack(fill="x", pady=(0, 12))
        self.target_selector = ttk.Combobox(targets, textvariable=self.target_choice,
            values=tuple(TARGET_MODES.values()), state="readonly", width=18)
        self.target_selector.pack(side="left", padx=(0, 12))
        self.target_selector.bind("<<ComboboxSelected>>", self._change_target_mode)
        ttk.Label(targets, text="随机种子").pack(side="left")
        ttk.Entry(targets, textvariable=self.target_seed, width=12).pack(side="left", padx=6)
        ttk.Button(targets, text="应用目标 / 种子", command=self._change_target_mode).pack(side="left", padx=6)
        if self.sim.launch_profile is None:
            self.target_selector.configure(state="disabled")
        noise = ttk.LabelFrame(host, text="IMU噪声", padding=10)
        noise.pack(fill="x", pady=(0, 12))
        ttk.Checkbutton(noise, text="启用噪声", variable=self.imu_noise_enabled).pack(side="left")
        ttk.Label(noise, text="倍率").pack(side="left", padx=(18, 5))
        ttk.Entry(noise, textvariable=self.imu_noise_scale, width=10).pack(side="left")
        ttk.Button(noise, text="应用噪声并重置", command=self._apply_imu_noise).pack(side="left", padx=12)
        ttk.Label(host, text="质量、气动、舵效与靶板材料仍为估计值；修改参数不会自动校准实物能力。",
                  style="Muted.TLabel", wraplength=950).pack(anchor="w", pady=6)

    def _build_onboard_window(self) -> None:
        """相机端独立窗口；隐藏不销毁，不影响主窗口控制与采样。"""
        self.onboard_window = tk.Toplevel(self.root)
        self.onboard_window.title("镖载视觉 · 与第三视角同步")
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        width = min(620, max(500, screen_width-60))
        height = min(760, max(570, screen_height-100))
        self.onboard_window.geometry(
            f"{width}x{height}+{max(22, screen_width-width-28)}+28")
        self.onboard_window.minsize(460, min(540, height))
        self.onboard_window.configure(background=COLORS["background"])
        self.onboard_window.protocol("WM_DELETE_WINDOW", self.hide_onboard)
        self.onboard_window.bind("<space>", self._space_key)
        self.onboard_window.bind("<Escape>", lambda _event: self.hide_onboard())
        outer = ttk.Frame(self.onboard_window, padding=(10, 9))
        outer.pack(fill="both", expand=True)
        header = ttk.Frame(outer)
        header.pack(fill="x", pady=(0, 7))
        ttk.Label(header, text="镖载视觉", style="Section.TLabel").pack(side="left")
        ttk.Label(header, textvariable=self.onboard_clock, style="Muted.TLabel",
                  font=("Microsoft YaHei UI", 9)).pack(side="right")
        self.onboard_tabs = ttk.Notebook(outer)
        self.onboard_tabs.pack(fill="both", expand=True)
        self.onboard_image_page = ttk.Frame(self.onboard_tabs, padding=(0, 6, 0, 0))
        self.onboard_settings_page = ttk.Frame(self.onboard_tabs)
        self.onboard_tabs.add(self.onboard_image_page, text="相机画面")
        self.onboard_tabs.add(self.onboard_settings_page, text="视觉设置")
        self.onboard_image_page.columnconfigure(0, weight=1)
        self.onboard_image_page.rowconfigure(1, weight=1)
        image_controls = ttk.Frame(self.onboard_image_page)
        image_controls.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        image_selector = ttk.Combobox(image_controls, textvariable=self.image_choice,
            values=("识别叠加", "相机图（光晕/曝光后）", "MuJoCo原始图", "LAB阈值掩膜"),
            state="readonly", width=25)
        image_selector.pack(side="left")
        image_selector.bind("<<ComboboxSelected>>", lambda _event: self._request_render())
        ttk.Button(image_controls, text="视觉参数", command=lambda: self._show_settings("vision")).pack(
            side="right")
        self.onboard_label = tk.Label(self.onboard_image_page, bg=COLORS["viewport"], bd=0)
        self.onboard_label.grid(row=1, column=0, sticky="nsew")
        self.onboard_label.bind("<Configure>", lambda _event: self._request_render())
        self.onboard_label.bind("<Button-1>", self._onboard_click)
        self.camera_summary = tk.StringVar(value="相机等待采样")
        camera_status = ttk.Label(self.onboard_image_page, textvariable=self.camera_summary,
                                  style="Muted.TLabel", wraplength=560)
        camera_status.grid(row=2, column=0, sticky="w", pady=(7, 0))
        self.onboard_image_page.bind("<Configure>", lambda event: camera_status.configure(
            wraplength=max(180, event.width-12)), add="+")
        ttk.Label(self.onboard_image_page, textvariable=self.vision_status, style="Muted.TLabel",
                  font=("Microsoft YaHei UI", 9), wraplength=560).grid(
                      row=3, column=0, sticky="w", pady=(7, 0))
        self.vision_settings_host = self._scrolling_settings(self.onboard_settings_page)
        self._build_vision_settings(self.vision_settings_host)

    def _build_vision_settings(self, host) -> None:
        ttk.Label(host, text="视觉识别", style="Section.TLabel").pack(anchor="w")
        ttk.Label(host, text="切换到相机画面点选参考，再应用参数重新开始本轮。",
                  style="Muted.TLabel", wraplength=460).pack(anchor="w", pady=(4, 12))
        settings = ttk.LabelFrame(host, text="阈值与成像近似", padding=10)
        settings.pack(fill="x")
        row = ttk.Frame(settings)
        row.pack(fill="x")
        profiles = ttk.Combobox(row, textvariable=self.vision_profile,
                               values=("仿真绿灯", "原OpenMV"), state="readonly", width=12)
        profiles.pack(side="left")
        profiles.bind("<<ComboboxSelected>>", self._fill_vision_profile)
        ttk.Label(row, text="曝光近似 µs").pack(side="left", padx=(18, 5))
        ttk.Entry(row, textvariable=self.exposure_entry, width=9).pack(side="left")
        thresholds = ttk.Frame(settings)
        thresholds.pack(fill="x", pady=(10, 0))
        for channel, label in enumerate(("L", "a", "b")):
            ttk.Label(thresholds, text=label).pack(side="left", padx=(18 if channel else 0, 6))
            for bound in range(2):
                ttk.Entry(thresholds, textvariable=self.lab_entries[channel*2+bound], width=6).pack(
                    side="left", padx=2)
        bloom = ttk.Frame(settings)
        bloom.pack(fill="x", pady=(10, 0))
        ttk.Label(bloom, text="光晕半径 px").pack(side="left")
        ttk.Entry(bloom, textvariable=self.bloom_radius_entry, width=8).pack(side="left", padx=6)
        ttk.Label(bloom, text="光晕增益").pack(side="left", padx=(18, 0))
        ttk.Entry(bloom, textvariable=self.bloom_gain_entry, width=8).pack(side="left", padx=6)
        refs = ttk.LabelFrame(host, text="图像参考点", padding=10)
        refs.pack(fill="x", pady=(12, 0))
        for title, variables in (("红色瞄准参考", self.aim_point_entries),
                                 ("橙色机体参考", self.dart_reference_entries)):
            point_row = ttk.Frame(refs)
            point_row.pack(fill="x", pady=(0, 7))
            ttk.Label(point_row, text=title, width=15).pack(side="left")
            for axis, variable in zip(("X", "Y"), variables):
                ttk.Label(point_row, text=axis).pack(side="left", padx=(8, 4))
                ttk.Entry(point_row, textvariable=variable, width=7).pack(side="left")
        pick = ttk.Frame(refs)
        pick.pack(fill="x")
        ttk.Label(pick, text="图像点选", width=15).pack(side="left")
        ttk.Combobox(pick, textvariable=self.cursor_pick_mode, state="readonly", width=18,
                     values=("不点选", "红色瞄准参考", "橙色机体参考")).pack(side="left", padx=8)
        ttk.Button(pick, text="返回相机点选", command=self.show_onboard).pack(side="left", padx=8)
        if getattr(self.sim, "_aim_reference_config", None) is not None:
            reference_row = ttk.Frame(refs)
            reference_row.pack(fill="x", pady=(8, 0))
            ttk.Label(reference_row, text="红十字用途", width=15).pack(side="left")
            ttk.Combobox(reference_row, textvariable=self.aim_reference_mode, state="readonly", width=19,
                values=("仅显示参考", "参与靶面微调")).pack(side="left", padx=8)
            reference_notice = "参与微调：标定视角下绿灯期望位置；下移使目标抬高，右移使目标向左。仅显示参考时不改变目标。"
        elif getattr(self.sim, "expert_config", {}).get("law") == "ballistic_pn":
            reference_notice = "本入口红十字仅显示，专家按上方靶心制导。需要参考微调请使用v3入口。"
        else:
            gains = self.sim.flight_config["controller"]
            reference_notice = (f"红点额外瞄准增益：俯仰{gains['aim_pitch_gain_g_per_deg']:g}、"
                f"偏航{gains['aim_yaw_gain_g_per_deg']:g} g/度；增益为0时不参与控制。")
        ttk.Label(refs, text="源图320×240，左上角(0,0)。橙点仅显示。"+reference_notice,
                  style="Muted.TLabel", wraplength=460).pack(anchor="w", pady=(8, 0))
        ttk.Button(host, text="应用视觉并重置实验", command=self._apply_vision_settings,
                   style="Accent.TButton").pack(anchor="w", pady=(12, 0))
        ttk.Label(host, text="OpenCV桌面近似；曝光、光晕尚未按OV7725实测标定，UART仅显示数据包。",
                  style="Muted.TLabel", wraplength=460).pack(anchor="w", pady=(12, 0))

    def _build_details(self, host) -> None:
        for title, variable in (("运动与命中", self.metrics), ("飞控与过载", self.flight_telemetry),
                                ("视觉识别", self.vision_status), ("相机时钟", self.onboard_clock)):
            group = ttk.LabelFrame(host, text=title, padding=12)
            group.pack(fill="x", pady=(0, 12))
            label = ttk.Label(group, textvariable=variable, justify="left", wraplength=1100)
            label.pack(fill="x", anchor="w")
            group.bind("<Configure>", lambda event, item=label: item.configure(
                wraplength=max(300, event.width-28)))

    def _onboard_click(self, event) -> None:
        """反算居中缩放图中的源坐标；仅填待应用框，不产生视觉或控制帧。"""
        mode = self.cursor_pick_mode.get()
        variables = (self.aim_point_entries if mode == "红色瞄准参考" else
                     self.dart_reference_entries if mode == "橙色机体参考" else None)
        rect = self._image_rects.get("onboard")
        if variables is None or rect is None:
            return
        x, y, width, height, source_width, source_height = rect
        if not (x <= event.x < x+width and y <= event.y < y+height):
            return
        u = min(source_width-1, math.floor((event.x-x)*source_width/width))
        v = min(source_height-1, math.floor((event.y-y)*source_height/height))
        variables[0].set(str(u))
        variables[1].set(str(v))
        self.message.set(f"{mode}待应用坐标 ({u}, {v})；点击“应用视觉并重置实验”生效。")

    def _close_flight_tuning(self) -> None:
        panel = getattr(self, "_flight_tuning_panel", None)
        if panel is not None:
            panel.close()
            self._flight_tuning_panel = None
        monitor = self._flight_debug_window
        if monitor is not None:
            for axis, host in monitor._tuning_hosts.items():
                for child in host.winfo_children():
                    child.destroy()
                host.pack_forget()
                monitor._tune_buttons[axis].configure(text="调参")
        self._axis_tuning_hints.clear()
        self._axis_tuning_signatures.clear()

    def _open_flight_tuning(self) -> None:
        self._show_settings("flight")
        self._ensure_flight_tuning_panel()

    def _ensure_flight_tuning_panel(self):
        if self._flight_tuning_panel is None:
            for child in self.flight_tuning_host.winfo_children():
                child.destroy()
            self._flight_tuning_panel = FlightTuningPanel(
                self.root, self.sim, self._flight_tuning_applied, parent=self.flight_tuning_host)
        return self._flight_tuning_panel

    def _toggle_axis_tuning(self, axis):
        """The waveform editor shares the full panel's draft and validation."""
        monitor = self._flight_debug_window
        host = monitor._tuning_hosts[axis]
        if host.winfo_manager():
            host.pack_forget()
            monitor._tune_buttons[axis].configure(text="调参")
        else:
            panel = self._ensure_flight_tuning_panel()
            if not host.winfo_children():
                self._build_axis_tuning(axis, host, panel)
            host.pack(fill="x", before=monitor._canvases[axis], pady=(2, 4))
            monitor._tune_buttons[axis].configure(text="收起")
        self.root.after_idle(monitor._fit_stacked)
        monitor._invalidate()
        self.dirty = True
        self.message.set("波形就地调参：空白轴参数继承公共值；点击应用会统一应用待编辑参数并重置本轮。")

    def _build_axis_tuning(self, axis, host, panel):
        def entry(key, row, col, label):
            ttk.Label(host, text=label).grid(row=row, column=col, sticky="w", padx=(0, 2))
            widget = ttk.Entry(host, textvariable=panel.entries[("controller", key)], width=4)
            widget.grid(row=row, column=col+1, sticky="w", padx=(0, 2), pady=2)
            widget.bind("<Return>", lambda _event: panel.apply())
            widget.bind("<FocusIn>", lambda _event: self.message.set(
                key+"；"+(self._axis_tuning_hints[axis].get() if axis in self._axis_tuning_hints
                          else "舵限留空按当前滚转模式继承。")+
                "；点击应用或按Enter会应用调参草稿并重置本轮。"))
        if axis == "roll":
            ttk.Label(host, text="模式").grid(row=0, column=0, sticky="w")
            ttk.Combobox(host, textvariable=panel.entries[("controller", "roll_mode")],
                         values=tuple(ROLL_MODE_LABELS.values()), state="readonly", width=20).grid(
                row=0, column=1, columnspan=5, sticky="w", pady=2)
            fields = (("roll_target_deg", "目标°"), ("roll_angle_gain_per_s", "外环"),
                      ("roll_rate_gain_s", "内环"), ("roll_rate_limit_deg_s", "限速"),
                      ("roll_saturation_deg", "舵限°"), ("roll_gain", "原P"))
            for index, (key, label) in enumerate(fields):
                entry(key, 1+index//3, (index%3)*2, label)
            apply_row, apply_column = 0, 6
        else:
            mode = self.sim.flight_config["controller"]["autopilot_mode"]
            self._axis_tuning_signatures[axis] = mode
            fields = ([(axis+"_png_N", "N"), ("pid_"+axis+"_kp_deg_per_g", "Kp"),
                       ("pid_"+axis+"_ki_deg_per_g_s", "Ki"), ("pid_"+axis+"_kd_deg_s_per_g", "Kd")]
                      if mode == "acceleration_pid" else
                      [(axis+"_png_N", "N"), (axis+"_k_rate", "Kr"),
                       (axis+"_k_alpha", "Ka"), (axis+"_k_int", "Ki")])
            for index, (key, label) in enumerate(fields):
                entry(key, 0, index*2, label)
            self._axis_tuning_hints[axis] = tk.StringVar(master=self.root)
            apply_row, apply_column = 0, 8
        ttk.Button(host, text="应用", width=3, command=panel.apply, style="Small.TButton").grid(
            row=apply_row, column=apply_column, sticky="w", padx=(4, 0))
        self._refresh_axis_tuning_hints()

    def _refresh_axis_tuning_hints(self):
        config = self.sim.flight_config["controller"]
        for variable in self._axis_tuning_hints.values():
            variable.set(("N留空继承公共值；本轴PID独立参数" if config["autopilot_mode"] == "acceleration_pid" else
                "空白继承 N={:g} Kr={:g} Ka={:g} Ki={:g}".format(
                    config["png_N"], config["k_rate"], config["k_alpha"], config["k_int"])))

    def _flight_tuning_applied(self) -> None:
        mode = self.sim.flight_config["controller"]["autopilot_mode"]
        monitor = self._flight_debug_window
        if monitor is not None:
            for axis in tuple(self._axis_tuning_signatures):
                if self._axis_tuning_signatures[axis] != mode:
                    host = monitor._tuning_hosts[axis]
                    for child in host.winfo_children():
                        child.destroy()
                    self._build_axis_tuning(axis, host, self._flight_tuning_panel)
            self.root.after_idle(monitor._fit_stacked)
        self._refresh_axis_tuning_hints()
        self._new_recording()
        self.message.set("导引/飞控参数已应用并回到起点；飞控开关与第三视角保留。点击“开始”验证过载跟踪。")

    def show_onboard(self) -> None:
        """显示独立相机窗口；不改变运行、回放或物理状态。"""
        if self.closed:
            return
        self.onboard_window.deiconify()
        self.onboard_window.lift()
        self.onboard_tabs.select(self.onboard_image_page)
        self.dirty = True

    def hide_onboard(self) -> None:
        """关闭相机标题栏只隐藏；主窗口继续飞行与采样。"""
        if self.closed:
            return
        self.onboard_window.withdraw()
        self.message.set("相机窗口已隐藏，可点击“显示第一视角”重新打开。")

    def _fill_parameters(self) -> None:
        """把本次仿真真正使用的参数填进输入框；这里不修改物理状态。"""
        for key, variable in self.entries.items():
            variable.set(f"{float(self.sim.cfg[key]):g}")

    def _space_key(self, _event: Any) -> str | None:
        """空格切换运行状态；输入框中编辑文字时不抢走空格按键。"""
        if isinstance(self.root.focus_get(), (tk.Entry, ttk.Entry)):
            return None
        self.toggle_running()
        return "break"

    def _request_render(self) -> None:
        """窗口尺寸改变时标记重绘，等下一次界面调度统一处理。"""
        self.dirty = True

    def _change_external_view(self, _event: Any = None) -> None:
        """把中文选项转换成后端视角名称，并请求重新绘制当前时刻。"""
        view = {"场地全景": "overview", "跟随镖体": "follow",
                "基地后方看发射架": "base"}[self.view_choice.get()]
        self.sim.set_external_view(view)
        self.dirty = True

    def _reset_external_camera(self) -> None:
        """恢复下拉框所选预设的方向和缩放。"""
        self._change_external_view()

    def _camera_drag_start(self, event: Any) -> None:
        """记录鼠标起点；后续拖动仅改变第三视角相机。"""
        camera = self.sim.external_camera
        self._drag_origin = (event.x, event.y, float(camera.azimuth), float(camera.elevation))

    def _camera_drag_move(self, event: Any) -> None:
        """左键拖动旋转相机；不修改仿真中的物体位置或速度。"""
        if self._drag_origin is None:
            return
        x0, y0, azimuth, elevation = self._drag_origin
        camera = self.sim.external_camera
        camera.azimuth = azimuth + (event.x - x0) * .45
        camera.elevation = max(-89., min(-3., elevation - (event.y - y0) * .35))
        self.dirty = True

    def _camera_zoom(self, direction: int) -> None:
        """缩放第三视角；以相机距离的比例变化，近景和远景都容易控制。"""
        camera = self.sim.external_camera
        camera.distance = max(.25, min(80., camera.distance * (0.88 if direction > 0 else 1.14)))
        self.dirty = True

    def _camera_wheel(self, event: Any) -> str:
        """Windows 滚轮向前放大、向后缩小。"""
        self._camera_zoom(1 if event.delta > 0 else -1)
        return "break"

    def toggle_running(self) -> None:
        """开始或暂停；保留镖体状态，并重新建立真实时间的计时起点。"""
        if self.sim.done and not self.replaying:
            self.start_replay()
            return
        self.running = not self.running
        # 暂停期间的真实时间不能补算成飞行时间，所以清除待计算的余量。
        self.accumulator = 0.0
        self.last_wall = time.perf_counter()
        self.play_button.configure(text="暂停" if self.running else "开始")
        self.state_label.set("运行中" if self.running else "暂停中")
        self.message.set("正在运行。可随时暂停查看画面。" if self.running else "已暂停，仿真时间停止。")

    def _change_speed(self, _event=None) -> None:
        """倍率只改变时间推进的快慢，不改变物理步长或每仿真秒的采样数。"""
        self.accumulator = 0.
        self.last_wall = time.perf_counter()

    def _change_scene(self, _event=None) -> None:
        """成功建好新场地后再释放旧场地，并重新应用对应的维护中预设。"""
        if __package__:
            from .dart_demo import DemoSimulation, load_config
            from .simulation_setup import resolve_startup_paths
        else:
            from dart_demo import DemoSimulation, load_config
            from simulation_setup import resolve_startup_paths
        try:
            paths = resolve_startup_paths(scene=self.scene_paths[self.scene_choice.get()])
            factory = getattr(self.sim, "simulation_factory", DemoSimulation)
            replacement = factory(load_config(paths.config), paths.scene, paths.dart, paths.launch,
                target_mode=self.sim.target_mode if paths.launch else "fixed", target_seed=self.sim.target_seed,
                aero_model=self.sim.aero_model, flight_control_enabled=self.sim.flight_control_enabled,
                flight_control_config=copy.deepcopy(self.sim.flight_config))
            if paths.explicit_values:
                replacement.reset(dict(replacement.cfg, **paths.explicit_values))
        except (ValueError, OSError) as exc:
            messagebox.showerror("场地未切换", str(exc), parent=self.root)
            return
        self._close_flight_tuning()
        self.sim.close()
        self.sim = replacement
        self.scene_label.configure(text=self.sim.scene_label)
        self.view_selector.configure(values=(("场地全景", "跟随镖体", "基地后方看发射架")
                                            if self.sim.launch_profile else ("场地全景", "跟随镖体")))
        self.view_choice.set({"overview": "场地全景", "follow": "跟随镖体", "base": "基地后方看发射架"}[
            self.sim.external_view])
        self._fill_parameters()
        self.flight_control_choice.set("开启飞控" if self.sim.flight_control_enabled else "关闭飞控")
        self.target_choice.set(TARGET_MODES[self.sim.target_mode])
        self.target_seed.set(str(self.sim.target_seed))
        self.target_selector.configure(state="readonly" if self.sim.launch_profile else "disabled")
        self._new_recording()
        season = self.sim.field_scene.source.get("season", "当前") if self.sim.field_scene else "当前"
        self.message.set(f"已切换至 {season} 场地；{TARGET_MODES[self.sim.target_mode]}，保持原发射初值，并暂停。"
                         if self.sim.launch_profile else
                         "已切换场地并暂停。当前场地没有自动发射预设，使用演示起点。")

    def _change_flight_control(self, _event=None) -> None:
        """切换飞控重开本轮，避免在同一条记录里混合两种动力学。"""
        old_enabled = self.sim.flight_control_enabled
        try:
            selected = self.flight_control_choice.get()
            if selected not in ("开启飞控", "关闭飞控"):
                raise ValueError("请选择开启飞控或关闭飞控。")
            self.sim.set_flight_control(selected == "开启飞控")
        except (TypeError, ValueError, OSError) as exc:
            self.flight_control_choice.set("开启飞控" if old_enabled else "关闭飞控")
            messagebox.showerror("飞控未切换", str(exc), parent=self.root)
            return
        self._new_recording()
        self.message.set(f"已{selected}并回到起点；第三视角保留。点击“开始”观察本轮运动。")

    def _fill_imu_noise(self) -> None:
        """把生效配置回填控件；编辑中的文本不会直接改变传感器。"""
        noise = self.sim.flight_config["imu_noise"]
        self.imu_noise_enabled.set(noise["enabled"])
        self.imu_noise_scale.set(f"{noise['scale']:g}")

    def _apply_imu_noise(self) -> None:
        """完整验证后应用噪声并重开本轮；失败恢复原配置显示。"""
        try:
            scale = float(self.imu_noise_scale.get())
            if not math.isfinite(scale) or scale < 0.:
                raise ValueError("IMU噪声倍率必须是大于等于零的有限数值。")
            self.sim.set_imu_noise(bool(self.imu_noise_enabled.get()), scale)
        except (TypeError, ValueError, OSError) as exc:
            self._fill_imu_noise()
            messagebox.showerror("IMU噪声未应用", str(exc), parent=self.root)
            return
        self._new_recording()
        noise = self.sim.flight_config["imu_noise"]
        description = f"开启，倍率 {noise['scale']:g}" if noise["enabled"] else "关闭"
        self.message.set(f"IMU噪声已{description}，并回到起点；飞控选择与第三视角保留。")

    def _change_target_mode(self, _event=None) -> None:
        """选择目标后重开本轮；同种子可复现，不自动求解新的瞄准参数。"""
        old_mode = self.sim.target_mode
        old_seed = self.sim.target_seed
        try:
            mode = next(key for key, label in TARGET_MODES.items() if label == self.target_choice.get())
            seed = int(self.target_seed.get())
            self.sim.set_target_mode(mode, seed)
        except (ValueError, StopIteration) as exc:
            self.target_choice.set(TARGET_MODES[old_mode])
            self.target_seed.set(str(old_seed))
            messagebox.showerror("目标未切换", str(exc), parent=self.root)
            return
        self._new_recording()
        self.message.set(f"已选择{TARGET_MODES[mode]}，种子 {seed}。t=0 为首镖离架；保持原发射初值，点击“开始”观察。")

    def _new_recording(self) -> None:
        """新参数或新场地开始全新记录，旧回放不能跨模型继续使用。"""
        self.running = self.replaying = False
        self.accumulator = 0.
        self.last_wall = time.perf_counter()
        self.tracker.reset()
        self.history.clear()
        self.diagnostics.clear()
        self._replay_frame_index = None
        self.vision_clock.reset(self.sim.t)
        self.vision_count = self.vision_detected = self.vision_missed = 0
        self.vision_work_s = self.active_wall_s = 0.
        self._live_state = self._live_vision = None
        self._fill_imu_noise()
        self._vision_source = self._vision_raw = self._vision_image = None
        self._vision_snapshot = {}
        self._vision_t = None
        self.play_button.configure(text="开始")
        self.state_label.set("暂停中")
        self._sample_vision()
        self._record_flight_debug()
        self._render()

    def _record_flight_debug(self, observation=None) -> None:
        """记录真实宏步时刻；显示、回放和保存不会调用采样器。"""
        self.diagnostics.append(self.sim.t, self.sim.observation() if observation is None else observation)

    def _open_flight_debug(self) -> None:
        self.workspace_tabs.select(self.flight_page)
        if self._flight_debug_window is None:
            self._flight_debug_window = FlightDebugWindow(
                self.root, self.diagnostics, lambda: float(self.sim.t),
                parent=self.debug_host, stacked=True,
                on_tune_axis=None if hasattr(self.sim, "expert_config") else self._toggle_axis_tuning)
        self._flight_debug_window.show()
        self._flight_debug_window.refresh(float(self.sim.t))

    def _sample_vision(self) -> None:
        """仅在新的采样时刻处理一张真实渲染图，并记录同一时刻的状态。"""
        started = time.perf_counter()
        self._vision_source = self.sim.render("onboard")
        self._vision_raw = self.tracker.prepare_image(self._vision_source)
        self._vision_image, self._vision_snapshot = self.tracker.process(self._vision_raw)
        self._vision_t = self.sim.t
        self._vision_snapshot["sample_t"] = self._vision_t
        # 新测量先送入飞控，再保存同一帧状态；重绘、保存和只读回放不会走这里。
        if not self.replaying:
            self.sim.submit_visual_measurement(self._vision_snapshot)
        self.vision_count += 1
        self.vision_detected += int(self._vision_snapshot["detected"])
        self.vision_work_s += time.perf_counter() - started
        self.history.append(self.sim.t, self.sim.capture_state(), self._vision_snapshot)

    def _fill_vision_profile(self, _event=None) -> None:
        """预设只填输入框；用户点击应用后才重置实验并采用新参数。"""
        name = "openmv" if self.vision_profile.get() == "原OpenMV" else "simulation"
        settings = VisionSettings.for_profile(name)
        for variable, value in zip(self.lab_entries, settings.lab):
            variable.set(str(value))
        self.exposure_entry.set(str(settings.exposure_us))
        self.bloom_radius_entry.set(str(settings.bloom_radius_px))
        self.bloom_gain_entry.set(str(settings.bloom_gain))

    def _apply_vision_settings(self) -> None:
        """先验证完整参数，再更新桌面视觉配置；不修改原 OpenMV 源文件。"""
        try:
            settings = VisionSettings(lab=tuple(int(v.get()) for v in self.lab_entries),
                                      exposure_us=int(self.exposure_entry.get()), profile="custom",
                                      bloom_radius_px=int(self.bloom_radius_entry.get()),
                                      bloom_gain=float(self.bloom_gain_entry.get()),
                                      aim_point_px=tuple(int(v.get()) for v in self.aim_point_entries),
                                      dart_reference_px=tuple(int(v.get()) for v in self.dart_reference_entries))
            for name in ("simulation", "openmv"):
                preset = VisionSettings.for_profile(name)
                if (settings.lab == preset.lab and settings.exposure_us == preset.exposure_us
                        and settings.bloom_radius_px == preset.bloom_radius_px
                        and settings.bloom_gain == preset.bloom_gain):
                    settings = replace(preset, aim_point_px=settings.aim_point_px,
                                       dart_reference_px=settings.dart_reference_px)
                    break
            # 先准备可复用配置文件；失败时不破坏已运行的实验。
            path = ROOT / "profiles" / "vision_desktop.json"
            if getattr(self.sim, "_aim_reference_config", None) is not None:
                self.sim.set_aim_reference_mode({"仅显示参考": "display_only", "参与靶面微调": "image_reference"}[self.aim_reference_mode.get()])
            path.write_text(json.dumps(settings.as_dict(), ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
            self.sim.reset()
            self.tracker.set_settings(settings)
        except (ValueError, TypeError, OSError) as exc:
            messagebox.showerror("视觉参数未应用", str(exc), parent=self.root)
            return
        self._new_recording()
        self.vision_profile.set({"simulation": "仿真绿灯", "openmv": "原OpenMV"}.get(settings.profile, "自定义"))
        reference = getattr(self.sim, "aim_reference_observation", {})
        self.message.set("视觉与光标已应用并回到起点；红点用途："+
            reference.get("mode", "传统额外瞄准项，是否生效由增益决定"))

    def _enter_replay(self) -> None:
        """第一次进入回放时封存最新 live 状态和画面；回放不触碰 tracker。"""
        if not self.replaying:
            self._live_state = self.sim.capture_state()
            self._live_vision = (self._vision_source, self._vision_raw, self._vision_image,
                                 copy.deepcopy(self._vision_snapshot), self._vision_t)
            self.replaying = True
            self._replay_frame_index = None
        self.accumulator = 0.
        self.last_wall = time.perf_counter()

    def _show_replay_frame(self, t: float) -> None:
        """恢复不晚于游标的一帧；绘图时只重画该帧已有的识别结果。"""
        index = self.history.index_at(t)
        if self._replay_frame_index == index:
            return
        frame = self.history.at(t)
        self.sim.restore_state(frame.state)
        self._replay_frame_index = index
        self._vision_snapshot = frame.vision
        self._vision_t = frame.t
        self._vision_source = self._vision_raw = self._vision_image = None
        self.dirty = True

    def start_replay(self) -> None:
        """从当前已记录内容的开头播放；未录制到下一帧时给出操作提示。"""
        if len(self.history) < 2:
            self.message.set("先点击“开始”运行一段，再从头回放或拖动时间轴。")
            return
        self._enter_replay()
        self.replay_cursor = self.history.start_time
        self._show_replay_frame(self.replay_cursor)
        self.running = True
        self.play_button.configure(text="暂停")
        self.state_label.set("回放中")
        self.message.set("正在回放已记录状态；可拖动时间轴或选择慢动作倍率。")

    def _seek_replay(self, value) -> None:
        """用户拖动时间轴后暂停定位；程序更新滑条不能递归触发定位。"""
        if self._updating_timeline or len(self.history) < 2:
            return
        self._enter_replay()
        self.replay_cursor = max(self.history.start_time, min(float(value), self.history.end_time))
        self._show_replay_frame(self.replay_cursor)
        self.running = False
        self.play_button.configure(text="开始")
        self.state_label.set("回放已暂停")
        self.message.set("已定位记录中的画面。点击“开始”继续回放，“回到最新”返回当前实验。")

    def return_live(self) -> None:
        """退出只读回放，恢复进入回放前的最新物理状态，不从中间重新积分。"""
        if self.replaying:
            self.sim.restore_state(self._live_state)
            (self._vision_source, self._vision_raw, self._vision_image,
             self._vision_snapshot, self._vision_t) = self._live_vision
        self.replaying = self.running = False
        self._replay_frame_index = None
        self.accumulator = 0.
        self.last_wall = time.perf_counter()
        self.play_button.configure(text="开始")
        self.state_label.set("本次已结束" if self.sim.done else "暂停中")
        self.message.set("已回到最新记录。可继续运行；重新应用参数会开始新一次录制。")
        self.dirty = True

    def reset(self) -> None:
        """读取输入框、验证范围，再重建本次实验；成功后回到起点暂停。

        输入框只是待应用设置，编辑时不会中途改变正在飞行的镖体。
        先构造 candidate 副本，确保无效输入不会破坏已经运行的实验。
        """
        candidate = dict(self.sim.cfg)
        try:
            for key, title, unit, lower, upper in PARAMETERS:
                number = float(self.entries[key].get())
                if not lower <= number <= upper:
                    raise ValueError(f"{title}应在 {lower:g} 到 {upper:g} {unit}之间。")
                candidate[key] = number
            self.sim.reset(candidate)
        except (TypeError, ValueError) as exc:
            messagebox.showerror("参数未应用", f"请检查输入：\n{exc}", parent=self.root)
            return
        self._new_recording()
        self.message.set("已应用参数并回到起点；第三视角保留。点击“开始”观察本次运动。")

    def _tick(self) -> None:
        """把真实经过的时间换算成整数个物理步，并独立限制画面刷新频率。

        例如真实经过约 0.010 秒、物理 dt=0.002 秒，就计算约 5 步。
        不足一个 dt 的余量留到下次累加；暂停时不执行任何物理步。
        root.after(10, ...) 只安排下次检查，不能保证电脑恰好准时执行。
        """
        if self.closed:
            return
        try:
            now = time.perf_counter()
            # 单次最多追赶 0.25 秒，防止窗口卡顿后积累大量任务而持续无响应。
            # 若机器长时间跟不上，演示会落后真实时间；物理 dt 本身不变化。
            wall_elapsed = max(0., now - self.last_wall)
            elapsed = min(wall_elapsed, 0.25)
            self.last_wall = now
            speed = float(self.speed_choice.get().replace("×", ""))
            if self.running and self.replaying:
                self.replay_cursor = min(self.history.end_time, self.replay_cursor + elapsed*speed)
                self._show_replay_frame(self.replay_cursor)
                if self.replay_cursor >= self.history.end_time:
                    self.return_live()
                    self.state_label.set("回放结束 · 最新状态")
            elif self.running and not self.sim.done:
                self.active_wall_s += wall_elapsed
                self.accumulator += elapsed*speed
                # 浮点累计可能让恰好一步变成 0.999999999 步；容差只消除舍入误差。
                count = int((self.accumulator + 1e-12) / self.sim.dt)
                if count:
                    completed = 0
                    deadline = time.perf_counter() + .010
                    # 必须逐宏步检查采样时刻，不能一次跳过多张相机画面再补重复帧。
                    for _ in range(count):
                        step_observation = self.sim.step()
                        completed += 1
                        crossed = self.vision_clock.consume(self.sim.t)
                        if crossed:
                            self.vision_missed += max(0, crossed-1)
                            self._sample_vision()
                            step_observation = None  # Measurement submission updated controller observation.
                        if self.sim.done:
                            if self._vision_t != self.sim.t:
                                self._sample_vision()  # 保存最后状态，允许回放到真实末端。
                            self._record_flight_debug()
                            break
                        self._record_flight_debug(step_observation)
                        if time.perf_counter() >= deadline:
                            break  # Yield to Tk; remaining physical steps stay in the accumulator.
                    self.accumulator = max(0., self.accumulator - completed*self.sim.dt)
                    self.dirty = True
                if self.sim.done:
                    self.running = False
                    self.accumulator = 0.0
                    self.play_button.configure(text="开始")
                    self.state_label.set("本次已结束")
                    state = self.sim.observation()["status"]
                    self.message.set(f"{STATUS_TEXT.get(state, state)}。可拖时间轴、从头回放或重置参数。")
            # 目标约每秒 30 帧；生成一帧可能对应前面已经完成的多次物理计算。
            if self.dirty and now - self.last_render >= 1.0 / 30.0:
                self._render()
                self.last_render = time.perf_counter()
            self._after_id = self.root.after(2, self._tick)
        except Exception as exc:
            self.running = False
            self.play_button.configure(text="开始")
            self.state_label.set("运行出错")
            self.message.set(f"运行失败：{exc}")
            messagebox.showerror("运行失败", str(exc), parent=self.root)
            self.close()

    def _display(self, camera: str, array: Any, label: tk.Label) -> None:
        """显示层独立缩放/HUD；原始相机像素绝不返回视觉识别器。"""
        picture = Image.fromarray(array)
        source_width, source_height = picture.size
        available = (max(1, label.winfo_width()), max(1, label.winfo_height()))
        # External rendering follows its viewport. The calibrated onboard source
        # always remains 320x240 and is fitted without changing its intrinsics.
        ratio = min(available[0] / picture.width, available[1] / picture.height)
        size = (max(1, round(picture.width * ratio)), max(1, round(picture.height * ratio)))
        self._image_rects[camera] = ((available[0]-size[0])/2, (available[1]-size[1])/2,
                                     *size, source_width, source_height)
        if picture.size != size:
            # 运行时使用便宜的双线性放大；暂停细看时再用高质量重采样。
            picture = picture.resize(size, Image.Resampling.BILINEAR if self.running
                                     else Image.Resampling.LANCZOS)
        viewport = Image.new("RGB", available, COLORS["viewport"])
        viewport.paste(picture, ((available[0]-size[0])//2, (available[1]-size[1])//2))
        target = self.g_hud_target.get()
        if ((camera == "external" and target == "第三视角") or
                (camera == "onboard" and target == "镖载视觉")):
            cutoff = float(self.sim.t if camera == "external" or self._vision_t is None else self._vision_t)
            record = self.diagnostics.at(cutoff)
            monitor = self._flight_debug_window
            visible, manual_range = None, None
            if monitor is not None and getattr(monitor, "_shown", False):
                _modes, visible, ranges, _window, _errors = monitor._settings()
                manual_range = ranges.get("g")
            viewport = render_g_hud(viewport, record, size=min(230, max(200, available[1]//3)),
                                    visible=visible, manual_range=manual_range)
        picture = viewport
        if not hasattr(self, "_display_pictures"):
            self._display_pictures = {}
        self._display_pictures[camera] = picture
        photo = ImageTk.PhotoImage(picture, master=self.root)
        self._photos[camera] = photo  # 保留 Python 引用，避免图片被回收后窗口变空。
        label.configure(image=photo)

    def _external_viewport(self):
        """Stable, bounded display resolution; resizing never rebuilds GL per pixel."""
        width = max(64, self.external_label.winfo_width())
        height = max(64, self.external_label.winfo_height())
        scale = min(1., 960./width, 600./height)
        wanted = (max(1, round(width*scale)), max(1, round(height*scale)))
        current = getattr(self.sim, "_external_renderer_size", None)
        if current is None or current == wanted:
            return wanted
        now = time.perf_counter()
        if wanted != self._external_pending_size:
            self._external_pending_size = wanted
            self._external_resize_deadline = now + .18
            if self._external_resize_after is not None:
                self.root.after_cancel(self._external_resize_after)
            self._external_resize_after = self.root.after(190, self._finish_external_resize)
        if now >= self._external_resize_deadline:
            return wanted
        return current

    def _finish_external_resize(self):
        self._external_resize_after = None
        if not self.closed:
            self.dirty = True

    def _render(self, *, force=False) -> None:
        """只显示缓存/回放画面；不执行物理步，也不推进视觉状态机。"""
        if force or self.external_label.winfo_ismapped():
            width, height = self._external_viewport()
            self._display("external", self.sim.render("external", width=width, height=height), self.external_label)
        show_onboard = force or self.onboard_label.winfo_ismapped()
        if show_onboard and self._vision_raw is None:
            self._vision_source = self.sim.render("onboard")
            self._vision_raw = self.tracker.prepare_image(self._vision_source)
            self._vision_image = self.tracker.annotate(self._vision_raw, self._vision_snapshot)
        if show_onboard:
            mode = self.image_choice.get()
            shown = (self._vision_source if mode == "MuJoCo原始图" else
                     self._vision_raw if mode == "相机图（光晕/曝光后）" else
                     self.tracker.mask_image(self._vision_raw) if mode == "LAB阈值掩膜" else self._vision_image)
            self._display("onboard", shown, self.onboard_label)
        obs = self.sim.observation()
        p = obs["p_W"]
        target_result = obs.get("target_result")
        contact = obs.get("target_contact")
        target = obs.get("target_state")
        # 显示有物理意义的毫米精度，不把数值求解的浮点残差宣传成实物精度。
        distance_text = ("小于 1 毫米" if target_result and target_result['miss_distance_m'] < .001
                         else f"{target_result['miss_distance_m']*1000:.1f} 毫米" if target_result else "")
        target_line = (f"\n过靶时间 {target_result['crossing_time_s']:.3f} 秒     "
                       f"距靶心 {distance_text}（几何判定）"
                       if target_result is not None else "")
        if contact is not None:
            # 接触会使镖头弹回，未穿过参考平面也可以真实碰到靶板。
            # 将接触记录与自由飞行过面检查分开显示，避免把反弹误判成没碰到。
            target_line += f"\n首次接触靶板 {contact['time_s']:.3f} 秒 · 接触后继续计算"
            if not contact.get("rule_eligible", True):
                target_line += " · 检测条件未满足"
        if target is not None:
            target_line += (f"\n{target['label']} · 横移 {target['offset_m']*1000:+.1f} 毫米 · "
                            + ("可检测" if target["hit_eligible"] else "移动中/检测未就绪"))
        flight = obs.get("flight_control", {})
        flight_line = "\n飞控：" + ("开启" if flight.get("enabled", self.sim.flight_control_enabled) else "关闭")
        if flight.get("status"):
            flight_line += f" · {flight['status']}"
        servo_angles = flight.get("servo_actual_deg")
        if servo_angles is not None:
            flight_line += " · 实际舵角 " + " / ".join(f"{float(angle):+.1f}°" for angle in servo_angles)
        noise = flight.get("imu_noise", self.sim.flight_config["imu_noise"])
        flight_line += (f" · IMU噪声 ×{noise['scale']:g}" if noise["enabled"] else " · IMU噪声关闭")
        self.metrics.set(
            f"仿真时间 {obs['t']:.3f} 秒     速度 {obs['speed']:.2f} 米 / 秒     高度 {obs['altitude']:.2f} 米\n"
            f"位置 XYZ：({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}) 米     "
            f"状态：{STATUS_TEXT.get(obs['status'], obs['status'])}" + target_line + flight_line
        )
        self.run_summary.set(
            f"t {obs['t']:.3f} s    速度 {obs['speed']:.2f} m/s    高度 {obs['altitude']:.2f} m"
        )
        self._update_flight_telemetry(flight)
        if self._flight_debug_window is not None:
            self._flight_debug_window.refresh(float(obs["t"]), force=not self.running)
        self.onboard_clock.set(f"相机采样 {self._vision_t or 0.:.3f} 秒 · 当前仿真 {obs['t']:.3f} 秒")
        vision = self._vision_snapshot
        self.camera_summary.set(
            f"{vision.get('status', '等待识别')}  ·  相机 {self._vision_t or 0.:.3f} s"
            f"  ·  源图320×240  ·  误差 {vision.get('error_px', [0, 0])} px"
        )
        sim_hz = max(0, self.vision_count-1) / max(self.history.end_time, 1e-9)
        wall_hz = max(0, self.vision_count-1) / max(self.active_wall_s, 1e-9)
        detection_rate = self.vision_detected / max(1, self.vision_count)
        self.vision_status.set(
            f"识别：{vision.get('status', '未知')} · 像素误差 {vision.get('error_px', [0, 0])} · "
            f"色块面积 {vision.get('area_px', 0)} px\n"
            f"目标 90 Hz · 实测 {sim_hz:.1f} 帧/仿真秒 · {wall_hz:.1f} 帧/现实秒\n"
            f"本轮检测比例 {detection_rate:.0%} · 采样跨格 {self.vision_missed}\n"
            f"{vision.get('reason', '')} · UART {vision.get('packet_hex') or '无发送'}"
        )
        self._updating_timeline = True
        try:
            self.timeline.configure(from_=self.history.start_time, to=max(.001, self.history.end_time))
            # 拖动请求可落在两视觉帧之间；显示游标与真正恢复的帧时刻一致。
            self.timeline.set(float(obs["t"]))
        finally:
            self._updating_timeline = False
        self.timeline_text.set(f"{'回放' if self.replaying else '当前运行'} {obs['t']:.3f} / "
                               f"已记录 {self.history.end_time:.3f} 秒 · {len(self.history)} 帧 · "
                               f"倍率 {self.speed_choice.get()}")
        self.dirty = False

    def _update_flight_telemetry(self, flight) -> None:
        """仅读取同一快照的过载；关闭时不把陈旧控制指令当成当前指令。"""
        enabled = flight.get("enabled", False)
        control = flight.get("control", {})
        actual = flight.get("actual_overload_g", {})
        ideal = flight.get("realization") == "ideal"
        lines = []
        for axis, name in (("pitch", "俯仰"), ("yaw", "偏航")):
            axis_on = enabled and self.sim.flight_config["controller"].get(f"{axis}_enabled", True)
            command = float(control.get(f"a_cmd_{axis}_g", 0.)) if axis_on else 0.
            measured = control.get(f"a_measured_{axis}_g")
            feedback = f"{measured:+.6f} g" if measured is not None else "尚未采样"
            demand_label = "PNG请求" if ideal else "期望"
            lines.append(f"{name} {'开启' if axis_on else '关闭'}  {demand_label} {command:+.3f} g   "
                         f"实际 {actual.get(axis, 0.):+.6f} g   IMU反馈(含符号) {feedback}")
        tick = control.get("control_time_s")
        if ideal:
            response = flight.get("response_model", {})
            limited = flight.get("limited_command_g", {})
            realized = flight.get("realized_control_g", {})
            version = "参考微调v3" if getattr(self.sim, "_aim_reference_config", None) is not None else "弹道预测PN专家v2"
            mode = (version if flight.get("guidance_law") == "ballistic_pn" else "传统PN对照")+" · 有限过载响应"
            lines.append(f"执行目标(下/右) {limited.get('pitch', 0.):+.3f}/{limited.get('yaw', 0.):+.3f} g · "
                         f"控制实际 {realized.get('pitch', 0.):+.3f}/{realized.get('yaw', 0.):+.3f} g")
            lines.append(f"合成上限 {response.get('max_control_g', 0.):.3f} g · "
                         f"τ={response.get('response_tau_s', 0.):.3f} s · "
                         f"变化率≤{response.get('max_slew_g_s', 0.):.2f} g/s · "
                         f"重力开启 / 阻力{'开启' if response.get('drag_enabled') else '关闭'}")
            prediction = flight.get("expert", {})
            reference = flight.get("aim_reference", {})
            if reference.get("mode") == "image_reference":
                right_up = reference.get("offset_plane_right_up_m", [0., 0.])
                lines.append(f"红十字目标微调：右 {right_up[0]*1000:+.2f} / 上 {right_up[1]*1000:+.2f} mm · "
                    +( "超出原靶面判定范围" if not reference.get("within_original_target_radius", True) else "原靶面范围内"))
            if "predicted_unforced_miss_m" in prediction:
                lines.append(f"预测零控制偏靶 {prediction['predicted_unforced_miss_m']*1000:.1f} mm · "
                             f"剩余 {prediction.get('time_to_go_s', 0.):.3f} s · {prediction.get('status', '')}")
        else:
            mode = "过载PID（对照）" if control.get("autopilot_mode") == "acceleration_pid" else "伪攻角三回路（v1）"
            def yaw_axis(values):
                return (values[0]-values[1]+values[2]-values[3])/4.
            requested = yaw_axis(flight.get("servo_command_deg", [0.]*4))
            realized = yaw_axis(flight.get("servo_actual_deg", [0.]*4))
            lines.append(f"偏航等效舵：指令 {requested:+.4f}° / 实际 {realized:+.4f}°")
            attitude = flight.get("attitude_deg", {})
            rates = flight.get("angular_rate_deg_s", {})
            roll_mode = "角度—角速率" if control.get("roll_mode") == "angle_rate" else "原P环"
            roll_on = "运行" if enabled and control.get("roll_active", False) else "未工作"
            lines.append(f"滚转 {roll_mode} · {roll_on} · 目标 {control.get('roll_target_deg', 0.):+.2f}° / "
                         f"实际 {attitude.get('roll', 0.):+.2f}° · 机体X角速率 {rates.get('roll', 0.):+.2f}°/s")
        lines.append(f"偏航变化：机体 {flight.get('yaw_attitude_relative_deg', 0.):+.3f}° / "
                     f"航迹 {flight.get('yaw_trajectory_relative_deg', 0.):+.3f}°（相对发射方向）")
        force_scope = "非重力比力；无碰撞响应" if ideal else "实际比力含接触"
        lines.append(f"{mode} · 控制采样 {tick:.3f} s · {flight.get('status', 'disabled')} · {force_scope}"
                     if tick is not None else f"控制尚未运行；{force_scope}")
        self.flight_telemetry.set("\n".join(lines))

    def save_capture(self) -> None:
        """保存当前双画面、数字参数和场地来源，便于复现这次观察。

        Tkinter 回调在同一线程依次执行，保存期间不会插入 _tick 推进，
        所以两张图片和 state.json 对应同一仿真时刻；这不是飞行录像。
        """
        # 只补刷新显示；识别结果始终取最近采样，保存动作不产生新识别帧。
        current_t = float(self.sim.observation()["t"])
        self._render(force=True)
        capture_root = ROOT / "outputs"
        if getattr(self.sim, "_aim_reference_config", None) is not None:
            capture_root /= "reference_v3"
        elif getattr(self.sim, "expert_config", {}).get("law") == "ballistic_pn":
            capture_root /= "pn_expert_v2"
        folder = capture_root / ("dart_viewer_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
        try:
            folder.mkdir(parents=True, exist_ok=False)
            Image.fromarray(self.sim.render("external")).save(folder / "external.png")
            for camera, shown in getattr(self, "_display_pictures", {}).items():
                shown.save(folder / (camera+"_display.png"))
            Image.fromarray(self._vision_image).save(folder / "onboard.png")
            Image.fromarray(self._vision_raw).save(folder / "onboard_raw.png")
            Image.fromarray(self._vision_source).save(folder / "onboard_source.png")
            Image.fromarray(self.tracker.mask_image(self._vision_raw)).save(folder / "onboard_mask.png")
            (folder / "config.json").write_text(
                json.dumps(self.sim.cfg, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            (folder / "flight_config.json").write_text(
                json.dumps(self.sim.flight_config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            (folder / "vision_config.json").write_text(
                json.dumps(self.tracker.settings.as_dict(), ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
            obs = self.sim.observation()
            state = {
                "t": float(obs["t"]),
                "status": obs["status"],
                "p_W": [float(value) for value in obs["p_W"]],
                "v_W": [float(value) for value in obs["v_W"]],
                "engine": getattr(self.sim, "engine_label", "MuJoCo"),
                "graphics_renderer": self._graphics_renderer(),
                "scene": self.sim.scene_metadata,
                "dart": self.sim.dart_metadata,
                "launch": self.sim.launch_metadata,
                "aerodynamics": self.sim.aero_metadata,
                "aero": obs.get("aero"),
                "flight_control_model": self.sim.flight_control_metadata,
                "flight_control": obs.get("flight_control"),
                "target_result": obs.get("target_result"),
                "target_contact": obs.get("target_contact"),
                "vision": self._vision_snapshot,
                "vision_sample_t": self._vision_t,
                "mode": "replay" if self.replaying else "live",
                "vision_sampling": {"target_hz": 90., "processed_frames": self.vision_count,
                                    "missed_grid_slots": self.vision_missed,
                                    "active_wall_s": self.active_wall_s},
            }
            (folder / "state.json").write_text(
                json.dumps(state, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            self.diagnostics.export_csv(folder / "flight_debug.csv", until=current_t)
            self.message.set(f"已保存当前两幅画面、本次配置及同一时刻的状态：{folder}")
        except (OSError, TypeError, ValueError) as exc:
            messagebox.showerror("保存失败", str(exc), parent=self.root)

    def _graphics_renderer(self) -> str:
        """读取后端实际查询到的 OpenGL 设备名，不根据电脑型号推测显卡。"""
        return str(getattr(self.sim, "graphics_renderer", None) or "unknown")

    def close(self) -> None:
        """取消下一次调度，再关闭渲染资源和窗口；重复调用直接返回。"""
        if self.closed:
            return
        self.closed = True
        self.running = False
        if self._after_id is not None:
            self.root.after_cancel(self._after_id)
        if self._external_resize_after is not None:
            self.root.after_cancel(self._external_resize_after)
        try:
            self._close_flight_tuning()
            if self._flight_debug_window is not None:
                self._flight_debug_window.close()
            self.sim.close()
        finally:
            self.root.destroy()


def main(argv: list[str] | None = None) -> int:
    """读取启动参数，加载场景，建立中文窗口，并交给 Tkinter 处理事件。"""
    parser = argparse.ArgumentParser(description="镖体运动双画面观察器（按场地加载仿真发射预设）")
    parser.add_argument("--config", type=Path, help="自定义演示配置 JSON 路径")
    parser.add_argument("--scene", type=Path, help="场地 manifest.json；省略时优先使用 2027 轻量场地")
    parser.add_argument("--season", choices=("2027",), help="选择维护中的 2027 场地；归档场地请使用 --scene 和 --launch")
    parser.add_argument("--dart", type=Path, help="镖体 manifest.json；省略时自动寻找深大镖")
    parser.add_argument("--launch", type=Path, help="发射方案 JSON；省略时按已登记场地寻找对应赛季固定靶预设")
    parser.add_argument("--auto-run", action="store_true", help="打开后自动开始")
    parser.add_argument("--target-mode", choices=tuple(TARGET_MODES), default="fixed", help="基地目标模式")
    parser.add_argument("--target-seed", type=int, default=2027, help="目标随机种子；相同种子重放相同运动")
    parser.add_argument("--aero-model", choices=("estimated_passive", "none"), default="estimated_passive",
                        help="气动后端；默认使用未标定的被动估计模型，none 仅关闭气动")
    parser.add_argument("--flight-control", choices=("on", "off"), default="off",
                        help="开启或关闭 Guidedemo 飞控，默认关闭；切换会重置本轮")
    parser.add_argument("--flight-config", type=Path, help="自定义飞控与估计舵效配置 JSON")
    parser.add_argument("--imu-noise-scale", type=float,
                        help="覆盖IMU噪声倍率；0关闭，大于0开启；省略时沿用飞控配置")
    parser.add_argument("--auto-close", type=float, help="指定秒数后自动退出，供本机界面检查使用")
    args = parser.parse_args(argv)
    if args.season and args.scene:
        parser.error("--season 与 --scene 选择其一即可")
    if args.season:
        args.scene = available_scenes().get(args.season)
        if args.scene is None:
            parser.error(f"{args.season} 场地尚未转换，请先生成场地资产")
    if args.auto_close is not None and args.auto_close <= 0:
        parser.error("--auto-close 必须大于 0")
    if args.imu_noise_scale is not None and (not math.isfinite(args.imu_noise_scale) or args.imu_noise_scale < 0):
        parser.error("--imu-noise-scale 必须是大于等于0的有限数值")
    if __package__:
        from .dart_demo import DemoSimulation, load_config, resolve_scene_manifest
        from .simulation_setup import resolve_startup_paths
    else:
        from dart_demo import DemoSimulation, load_config, resolve_scene_manifest
        from simulation_setup import resolve_startup_paths
    try:
        inputs = resolve_startup_paths(args.config, args.scene, args.dart, args.launch)
        simulation = DemoSimulation(load_config(inputs.config), inputs.scene, inputs.dart, inputs.launch,
            target_mode=args.target_mode, target_seed=args.target_seed, aero_model=args.aero_model,
            flight_control_enabled=args.flight_control == "on", flight_control_config=args.flight_config)
        if inputs.explicit_values:
            simulation.reset(dict(simulation.cfg, **inputs.explicit_values))
        if args.imu_noise_scale is not None:
            simulation.set_imu_noise(args.imu_noise_scale > 0., args.imu_noise_scale)
        vision_path = ROOT / "profiles" / "vision_desktop.json"
        vision_settings = (VisionSettings(**json.loads(vision_path.read_text(encoding="utf-8")))
                           if vision_path.is_file() else VisionSettings.for_profile("simulation"))
    except ValueError as exc:
        parser.error(str(exc))
    root: tk.Tk | None = None
    viewer: DartViewer | None = None
    try:
        root = tk.Tk()
        viewer = DartViewer(root, simulation, auto_run=args.auto_run, vision_settings=vision_settings)
        if args.auto_close is not None:
            root.after(round(args.auto_close * 1000), viewer.close)
        root.mainloop()  # 等待点击、键盘和 after 定时回调；物理计算由 _tick 触发。
    finally:
        if viewer is not None:
            viewer.close()
        else:
            simulation.close()
            if root is not None:
                root.destroy()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
