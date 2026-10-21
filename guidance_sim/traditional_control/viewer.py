"""Physical predictive-PN viewer with explicit guidance and autopilot editors.

The inherited viewer owns cameras, physics scheduling, diagnostics and replay.
This module only changes which draft parameters are presented and applied.
"""
from __future__ import annotations

import copy
import json
import math
from collections.abc import Mapping
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

# simulation and the entry point register prototypes on sys.path. Use that same
# module identity: loading a second prototypes.vision_settings would make its
# VisionSettings fail the tracker's isinstance check.
from dart_viewer import DartViewer
from guidedemo_control import load_flight_config
from tuning_panel import FlightTuningPanel
from .vision import ReacquiringTracker


PROJECT = Path(__file__).resolve().parents[1]
PREDICTION_FIELDS = (
    ("png_N", "预测PN比例 N", "无量纲"),
    ("vision_filter_tau_s", "目标估计滤波", "秒"),
    ("vision_timeout_s", "可信目标超时", "秒"),
    ("max_control_g", "预测请求合成限幅", "g；不是实物能力"),
    ("response_tau_s", "预测等效响应时间", "秒；不是实际舵机参数"),
    ("max_slew_g_s", "预测等效变化率", "g/秒；不直接限制实际比力"),
    ("update_period_s", "预测导引更新周期", "秒"),
    ("prediction_dt_s", "预测积分步长", "秒"),
)
INACTIVE_GUIDANCE_FIELDS = frozenset((
    "png_N", "pitch_png_N", "yaw_png_N", "closing_speed_mps",
    "aim_pitch_gain_g_per_deg", "aim_yaw_gain_g_per_deg", "aim_accel_limit_g",
    "pitch_los_sign", "yaw_los_sign", "pitch_gyro_compensation_sign",
    "yaw_gyro_compensation_sign", "pitch_guidance_sign", "yaw_guidance_sign",
))
ENTRY_NOTICE = (
    "本入口：预测PN → 真实自驾仪/PWM/舵机/气动。原PNG的N、接近速度和红点瞄准项不生效；"
    "预测N请在“预测导引（生效）”中修改。"
)


def validated_prediction_draft(base, numbers, *, drag_enabled):
    """Validate a complete UI draft without touching a simulation or Tk."""
    result = copy.deepcopy(dict(base))
    for key, text in numbers.items():
        if key not in result or key == "drag_enabled":
            raise ValueError(f"未知预测参数：{key}")
        if isinstance(text, bool):
            raise ValueError(f"{key}必须为正有限数字。")
        try:
            value = float(text)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"{key}必须为正有限数字。") from exc
        if not math.isfinite(value) or value <= 0.:
            raise ValueError(f"{key}必须为正有限数字。")
        result[key] = value
    if not isinstance(drag_enabled, bool):
        raise ValueError("预测阻力开关必须为布尔值。")
    result["drag_enabled"] = drag_enabled
    return result


class _PredictiveFlightPanel(FlightTuningPanel):
    """Retain the standard AP editor/export, label inactive legacy guidance."""

    def __init__(self, root, simulation, callback, *, owner, parent=None):
        self.owner = owner
        super().__init__(root, simulation, callback, parent=parent)
        self._heading_label.configure(text=ENTRY_NOTICE)
        # These inherited global actions always affect the AP draft, even when
        # the prediction page is selected. Name the boundary explicitly.
        names = {"应用并重置": "应用AP并重置", "保存JSON": "保存AP JSON", "加载JSON": "加载AP JSON"}
        def label_actions(widget):
            if isinstance(widget, ttk.Button):
                text = str(widget.cget("text"))
                if text in names:
                    widget.configure(text=names[text], width=13)
            for child in widget.winfo_children():
                label_actions(child)
        label_actions(self.window)
        for key in INACTIVE_GUIDANCE_FIELDS:
            widget = self.widgets.get(("controller", key))
            if widget is not None:
                widget.state(("disabled",))
        self.prediction_page = ttk.Frame(self.notebook)
        self.notebook.add(self.prediction_page, text="预测导引（生效）")
        host = owner._scrolling_settings(self.prediction_page)
        owner._build_prediction_controls(host)
        self.status.set("本页应用AP草稿；预测参数在独立页应用。原PNG N和红点修正已禁用。")

    def restore_current(self):
        super().restore_current()
        self.owner._fill_prediction_draft()
        self.status.set("已恢复当前AP和预测参数；未重置运行。")


class PhysicalTraditionalViewer(DartViewer):
    """Keep the inherited display/clock; expose the active physical controls."""

    tracker_factory = ReacquiringTracker

    def __init__(self, root, simulation, **kwargs):
        if not hasattr(simulation, "candidate_config"):
            raise TypeError("PhysicalTraditionalViewer需要带candidate_config的物理预测仿真。")
        self._prediction_entries = {
            key: tk.StringVar(master=root, value=f"{float(value):g}")
            for key, value in simulation.candidate_config.items() if key != "drag_enabled"
        }
        self._prediction_drag = tk.BooleanVar(
            master=root, value=simulation.candidate_config.get("drag_enabled", True))
        self._prediction_status = tk.StringVar(master=root, value="编辑预测草稿后单独应用；不修改真实舵效。")
        self._prediction_model_status = tk.StringVar(master=root)
        super().__init__(root, simulation, **kwargs)
        self._refresh_prediction_model()
        self.root.title("物理传统控制 · 预测PN与真实自驾仪")
        self.message.set(ENTRY_NOTICE)

    def _ensure_flight_tuning_panel(self):
        if self._flight_tuning_panel is None:
            for child in self.flight_tuning_host.winfo_children():
                child.destroy()
            self._flight_tuning_panel = _PredictiveFlightPanel(
                self.root, self.sim, self._flight_tuning_applied,
                owner=self, parent=self.flight_tuning_host)
        return self._flight_tuning_panel

    def _open_prediction_tuning(self):
        self._show_settings("flight")
        panel = self._ensure_flight_tuning_panel()
        panel.notebook.select(panel.prediction_page)

    def _build_prediction_controls(self, host):
        ttk.Label(host, text="共享预测导引参数（俯仰 / 偏航共用）",
                  style="Section.TLabel").pack(anchor="w")
        ttk.Label(host, text=(
            "此页N才参与预测PN。滤波作用于图像重建的上方靶心。\n"
            "合成限幅、响应时间和变化率属于预测模型；真实运动仍由自驾仪、PWM、舵机及气动力决定，"
            "修改这些数字不会提高实际舵效或实物过载能力。\n"
            "当前使用理想自身导航、静态灯靶标定与零风估计；预测总法向比力响应为等效近似。"),
            wraplength=720, justify="left").pack(anchor="w", pady=(6, 12))
        nominal = ttk.LabelFrame(host, text="只读标称预测模型", padding=(9, 7))
        nominal.pack(fill="x", pady=(0, 12))
        ttk.Label(nominal, textvariable=self._prediction_model_status, justify="left",
                  wraplength=700).pack(anchor="w")
        ttk.Label(nominal, text="发射与环境页修改的是实际镖体/空气参数；实际质量或Cd变化不会同步更新此标称模型。",
                  wraplength=700, foreground="#B23B3B").pack(anchor="w", pady=(5, 0))
        self._refresh_prediction_model()
        fields = ttk.Frame(host)
        fields.pack(fill="x")
        labels = {key: (label, unit) for key, label, unit in PREDICTION_FIELDS}
        for row, (key, variable) in enumerate(self._prediction_entries.items()):
            label, unit = labels.get(key, (key, "按候选配置定义"))
            ttk.Label(fields, text=label).grid(row=row, column=0, sticky="w", padx=(0, 12), pady=4)
            entry = ttk.Entry(fields, textvariable=variable, width=14)
            entry.grid(row=row, column=1, sticky="w", padx=(0, 12), pady=4)
            entry.bind("<Return>", lambda _event: self._apply_prediction_draft())
            ttk.Label(fields, text=unit, style="Muted.TLabel").grid(row=row, column=2, sticky="w")
        ttk.Checkbutton(host, text="预测器保留阻力估计（不改变实际气动开关）",
                        variable=self._prediction_drag).pack(anchor="w", pady=(9, 6))
        actions = ttk.Frame(host)
        actions.pack(fill="x", pady=(5, 6))
        ttk.Button(actions, text="应用预测并重置", command=self._apply_prediction_draft,
                   style="Accent.TButton").pack(side="left")
        ttk.Button(actions, text="恢复当前预测", command=self._fill_prediction_draft).pack(side="left", padx=7)
        ttk.Button(actions, text="导出完整方案JSON", command=self._export_traditional_bundle).pack(side="left")
        ttk.Label(host, textvariable=self._prediction_status, wraplength=720,
                  style="Muted.TLabel").pack(anchor="w", pady=(5, 0))
        ttk.Label(host, text=(
            "完整方案同时保存预测草稿和标准AP草稿；标准AP JSON仍由原面板“保存JSON”导出。"
            "所有草稿应用都会重新开始本轮，导出不会改变运行。"),
            wraplength=720, style="Muted.TLabel").pack(anchor="w", pady=(9, 0))

    def _prediction_draft(self):
        return validated_prediction_draft(self.sim.candidate_config,
            {key: variable.get() for key, variable in self._prediction_entries.items()},
            drag_enabled=bool(self._prediction_drag.get()))

    def _fill_prediction_draft(self):
        for key, variable in self._prediction_entries.items():
            variable.set(f"{float(self.sim.candidate_config[key]):g}")
        self._prediction_drag.set(self.sim.candidate_config.get("drag_enabled", True))
        self._prediction_status.set("已恢复当前生效的预测参数；未改变运行。")
        self._refresh_prediction_model()

    def _refresh_prediction_model(self):
        model = getattr(self.sim, "prediction_model", None)
        if not isinstance(model, Mapping):
            bundle = getattr(self.sim, "traditional_config", {})
            model = bundle.get("prediction_model") if isinstance(bundle, Mapping) else None
        if not isinstance(model, Mapping):
            self._prediction_model_status.set("未提供独立标称模型信息；不以当前实际参数代替标称值。")
            return
        fields = (("mass_kg", "m", "kg"), ("rho_kg_m3", "ρ", "kg/m³"),
                  ("area_m2", "S", "m²"), ("drag_coefficient", "Cd", ""))
        values = []
        for key, label, unit in fields:
            value = model.get(key, model.get("reference_area_m2") if key == "area_m2" else None)
            if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
                values.append(f"{label}={value:.8g} {unit}".strip())
            else:
                values.append(f"{label}=未提供")
        self._prediction_model_status.set("标称："+"；".join(values))

    def _new_recording(self):
        super()._new_recording()
        self._refresh_prediction_model()

    def _apply_imu_noise(self):
        """Rebase the AP form after an external noise change, preserving its safety."""
        try:
            scale = float(self.imu_noise_scale.get())
            if not math.isfinite(scale) or scale < 0.:
                raise ValueError("IMU噪声倍率必须是大于等于零的有限数值。")
        except (TypeError, ValueError, OverflowError):
            # Preserve the inherited error dialog and form recovery. No draft
            # is closed when input validation fails.
            return super()._apply_imu_noise()
        had_panel = self._flight_tuning_panel is not None
        if had_panel:
            notice = "应用IMU噪声会重置并关闭旧AP草稿，避免旧噪声值被再次应用；预测草稿保留。"
            self._flight_tuning_panel.status.set(notice)
            self.message.set(notice)
            self.root.update_idletasks()
        try:
            self.sim.set_imu_noise(bool(self.imu_noise_enabled.get()), scale)
        except (TypeError, ValueError, OSError) as exc:
            self._fill_imu_noise()
            messagebox.showerror("IMU噪声未应用", str(exc), parent=self.root)
            return
        if had_panel:
            self._close_flight_tuning()
        self._new_recording()
        noise = self.sim.flight_config["imu_noise"]
        description = f"开启，倍率 {noise['scale']:g}" if noise["enabled"] else "关闭"
        suffix = "旧AP草稿已关闭；再次调参将载入新噪声配置，预测草稿保留。" if had_panel else ""
        self.message.set(f"IMU噪声已{description}，并回到起点。"+suffix)

    def _apply_prediction_draft(self):
        try:
            self.sim.apply_candidate_config(self._prediction_draft())
        except (ValueError, TypeError, OSError) as exc:
            messagebox.showerror("预测参数未应用", str(exc), parent=self.root)
            return
        self._fill_prediction_draft()
        self._new_recording()
        self._prediction_status.set("预测参数已应用并重置；标准AP待编辑草稿未应用。")
        self.message.set("已应用预测PN参数并开始新记录；真实舵机、气动与AP系数保持各自生效配置。")

    def _traditional_bundle_draft(self):
        panel = self._ensure_flight_tuning_panel()
        bundle = copy.deepcopy(self.sim.traditional_config)
        if not isinstance(bundle, Mapping):
            raise ValueError("仿真未提供完整traditional_config方案。")
        bundle = dict(bundle)
        bundle["candidate"] = self._prediction_draft()
        bundle["flight_control"] = load_flight_config(panel._candidate())
        return bundle

    def _export_traditional_bundle(self):
        try:
            bundle = self._traditional_bundle_draft()
        except (ValueError, TypeError, OSError) as exc:
            messagebox.showerror("方案未导出", str(exc), parent=self.root)
            return
        path = filedialog.asksaveasfilename(parent=self.root, title="导出预测PN＋标准AP完整草稿",
            initialdir=str(PROJECT / "outputs"), initialfile="physical_traditional_tuned.json",
            defaultextension=".json", filetypes=(("JSON方案", "*.json"),), confirmoverwrite=True)
        if not path:
            return
        try:
            Path(path).write_text(json.dumps(bundle, ensure_ascii=False, indent=2, allow_nan=False)+"\n",
                                  encoding="utf-8")
        except (ValueError, TypeError, OSError) as exc:
            messagebox.showerror("方案未导出", str(exc), parent=self.root)
            return
        self._prediction_status.set(f"已导出两组待应用草稿：{path}；当前运行未改变。")

    def _build_axis_tuning(self, axis, host, panel):
        if axis == "roll":
            return super()._build_axis_tuning(axis, host, panel)
        mode = self.sim.flight_config["controller"]["autopilot_mode"]
        self._axis_tuning_signatures[axis] = mode
        if mode == "acceleration_pid":
            fields = [(f"pid_{axis}_kp_deg_per_g", "Kp"),
                      (f"pid_{axis}_ki_deg_per_g_s", "Ki"),
                      (f"pid_{axis}_kd_deg_s_per_g", "Kd"),
                      (f"pid_{axis}_rate_gain_s", "速率反馈")]
        else:
            fields = [(f"{axis}_k_rate", "Kr"), (f"{axis}_k_alpha", "Ka"),
                      (f"{axis}_k_int", "Ki"), (f"{axis}_filter_tau_s", "滤波τ")]
        for index, (key, label) in enumerate(fields):
            ttk.Label(host, text=label).grid(row=0, column=index*2, sticky="w", padx=(0, 2))
            entry = ttk.Entry(host, textvariable=panel.entries[("controller", key)], width=5)
            entry.grid(row=0, column=index*2+1, sticky="w", padx=(0, 4), pady=2)
            entry.bind("<Return>", lambda _event: panel.apply())
        ttk.Button(host, text="应用AP", command=panel.apply, style="Small.TButton").grid(
            row=1, column=0, columnspan=3, sticky="w", pady=(3, 0))
        ttk.Button(host, text="预测PN参数", command=self._open_prediction_tuning,
                   style="Small.TButton").grid(row=1, column=3, columnspan=5, sticky="e", pady=(3, 0))
        self._axis_tuning_hints[axis] = tk.StringVar(master=self.root)
        ttk.Label(host, textvariable=self._axis_tuning_hints[axis], wraplength=370,
                  style="Muted.TLabel").grid(row=2, column=0, columnspan=8, sticky="w", pady=(3, 0))
        self._refresh_axis_tuning_hints()

    def _refresh_axis_tuning_hints(self):
        mode = self.sim.flight_config["controller"]["autopilot_mode"]
        text = ("Kp °/g；Ki °/(g·s)；Kd °·s/g；速率反馈 s。预测N在独立参数页。"
                if mode == "acceleration_pid" else
                "Kr/Ka/Ki/滤波τ为真实AP参数，空白继承公共值；预测N在独立参数页。")
        for variable in self._axis_tuning_hints.values():
            variable.set(text)

    def _toggle_axis_tuning(self, axis):
        super()._toggle_axis_tuning(axis)
        self.message.set("波形编辑与完整面板共用AP草稿；应用AP会重置。预测N/滤波/限幅由“预测PN参数”单独应用。")

    def _build_vision_settings(self, host):
        super()._build_vision_settings(host)
        def visit(widget):
            if isinstance(widget, ttk.Label):
                text = str(widget.cget("text"))
                if "额外瞄准增益" in text:
                    widget.configure(text="源图320×240。此预测入口红十字和橙点仅显示；目标由绿灯测量及静态灯到上靶面的标定重建。")
            for child in widget.winfo_children():
                visit(child)
        visit(host)
        ttk.Label(host, text="本入口红十字不参与控制，修改它不会改变上方物理靶心或预测PN目标。",
                  wraplength=460, foreground="#B23B3B").pack(anchor="w", pady=(9, 0))

    def _update_flight_telemetry(self, flight):
        super()._update_flight_telemetry(flight)
        self.flight_telemetry.set(self.flight_telemetry.get()+"\n"+
            "物理预测PN入口：原PNG N与红点项无效；预测N/滤波在独立页，执行仍为真实AP/PWM/气动。")
