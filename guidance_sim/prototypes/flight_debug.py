"""Read-only three-axis flight diagnostics, Tk monitor and explicit CSV export.

The viewer owns sampling.  This module only consumes observations already taken;
it never calls a simulation, controller, tracker or random-number generator.
"""

from __future__ import annotations

from bisect import bisect_right
from collections import deque
import csv
from datetime import datetime
from functools import lru_cache
import json
import math
from numbers import Real
from pathlib import Path
from typing import Mapping
import time
import numpy as np

PALETTE = dict(background="#EDF3F8", surface="#FFFFFF", text="#23364A",
               muted="#556C7F", grid="#DFE8EF", command="#176B9A",
               actual="#25806C", feedback="#C47E28", warning="#B23B3B")
OUTPUTS = Path(__file__).resolve().parents[1] / "outputs" / "exports"
AXIS_LABELS = {"pitch": "俯仰", "yaw": "偏航", "roll": "滚转"}
MODES = {"过载": "g", "姿态角": "deg", "角速率": "deg/s", "等效舵量": "deg"}
SCHEMA_VERSION = 1


def _value(mapping, key, default=None):
    return mapping.get(key, default) if isinstance(mapping, Mapping) else default


def _numeric(value):
    if type(value) in (float, int):
        return True
    return isinstance(value, Real) and not isinstance(value, bool)


def _finite(value):
    if type(value) in (float, int):
        return math.isfinite(value)
    return _numeric(value) and math.isfinite(float(value))


def _flatten(prefix, obj, target, depth=0):
    """Keep scalar controller diagnostics without copying snapshots or arrays."""
    if depth > 4 or not isinstance(obj, Mapping):
        return
    for key, value in obj.items():
        name = f"{prefix}{key}"
        if isinstance(value, Mapping):
            _flatten(name + ".", value, target, depth+1)
        elif value is None or isinstance(value, (str, bool)) or _numeric(value):
            target[name] = float(value) if _numeric(value) else value


def _record(t, obs):
    flight = _value(obs, "flight_control", {}) or {}
    control = _value(flight, "control", {}) or {}
    result = dict(t_s=float(t), speed_mps=_value(obs, "speed"),
                  realization=_value(flight, "realization", "unknown"),
                  enabled=_value(flight, "enabled"), status=_value(flight, "status"),
                  autopilot_used=_value(flight, "autopilot_used"),
                  servo_model_used=_value(flight, "servo_model_used"),
                  command_limit_kind=_value(flight, "command_limit_kind"),
                  command_limit_g=_value(flight, "command_limit_g"),
                  dynamic_pressure_pa=_value(flight, "dynamic_pressure_pa"),
                  actual_time_s=_value(flight, "actual_time_s", _value(flight, "overload_time_s")))
    _flatten("control.", control, result)
    for axis in AXIS_LABELS:
        result[f"{axis}.actual_g"] = _value(_value(flight, "actual_overload_g"), axis)
        result[f"{axis}.command_g"] = _value(control, f"a_cmd_{axis}_g") if axis != "roll" else None
        result[f"{axis}.feedback_g"] = _value(control, f"a_measured_{axis}_g") if axis != "roll" else None
        result[f"{axis}.attitude_deg"] = _value(_value(flight, "attitude_deg"), axis)
        result[f"{axis}.rate_deg_s"] = _value(_value(flight, "angular_rate_deg_s"), axis)
        result[f"{axis}.axis_command_deg"] = _value(_value(flight, "axis_command_deg"), axis,
             _value(control, f"{axis}_cmd_deg")) if result["realization"] != "ideal" else None
        result[f"{axis}.axis_actual_deg"] = _value(_value(flight, "axis_actual_deg"), axis)
        imu = _value(control, "latest_imu", {}) or {}
        result[f"{axis}.imu_attitude_deg"] = _value(imu, f"{axis}_deg")
        result[f"{axis}.imu_rate_deg_s"] = _value(imu, f"{axis}_rate_deg_s")
        result[f"{axis}.limited_g"] = _value(_value(flight, "limited_command_g"), axis)
        result[f"{axis}.realized_control_g"] = _value(_value(flight, "realized_control_g"), axis)
    if result["realization"] == "ideal":
        result["roll.target_deg"] = None
        result["roll.rate_command_deg_s"] = None
    else:
        result["roll.target_deg"] = _value(control, "roll_target_deg")
        result["roll.rate_command_deg_s"] = _value(control, "roll_rate_command_deg_s")
    for key in ("actual_specific_force_L_g", "actuator_diagnostics", "imu_noise"):
        _flatten(key + ".", _value(flight, key, {}) or {}, result)
    for key, name in (("servo_command_deg", "command_deg"), ("servo_held_deg", "held_deg"),
                      ("servo_actual_deg", "actual_deg"), ("servo_pwm_us", "pwm_us")):
        array = _value(flight, key)
        for index in range(4):
            result[f"fin{index}.{name}"] = (array[index] if array is not None and len(array) > index else None)
    held = _value(flight, "servo_held_deg")
    if held is not None and len(held) == 4 and all(_finite(v) for v in held):
        a, b, c, d = (float(v) for v in held)
        for axis, val in zip(AXIS_LABELS, ((a+b-c-d)/4, (a-b+c-d)/4, (a-b-c+d)/4)):
            result[f"{axis}.axis_held_deg"] = val
    diag = _value(flight, "actuator_diagnostics", {}) or {}
    for field in ("mixed_command_exceeds_limit", "pwm_limited", "pwm_at_limit",
                  "held_at_mechanical_limit", "actual_at_mechanical_limit"):
        array = _value(diag, field)
        for index in range(4):
            result[f"fin{index}.{field}"] = (bool(array[index]) if array is not None and len(array) > index else None)
    for name in ("visual_measurement", "expert"):
        source = _value(flight, name, {}) or {}
        # Scalars include confidence/age/timestamps, but not image, target or trajectory arrays.
        _flatten(name + ".", source, result)
    last_valid = _value(control, "last_valid_visual_t_s")
    result["visual_age_s"] = max(0., float(t)-float(last_valid)) if _finite(last_valid) else None
    contact = _value(obs, "target_contact")
    contact_time = _value(contact, "time_s", _value(contact, "t_s", _value(contact, "t")))
    result["contact_time_s"] = contact_time
    result["contact_recorded"] = bool(contact)
    result["invalid_numeric_count"] = sum(_numeric(v) and not _finite(v) for v in result.values())
    return result


def column_metadata(name):
    """Units and provenance for exported columns, including raw controller fields."""
    units = "unspecified (raw diagnostic/configuration)"
    leaf = name.rsplit(".", 1)[-1]
    if leaf == "k_rate":
        units = "s"
    elif leaf == "k_int":
        units = "deg/(g*s)"
    elif leaf in ("k_alpha", "png_N", "filter_k_alpha", "filter_k_theta"):
        units = "1"
    elif leaf.endswith("_angle_gain_per_s"):
        units = "1/s"
    elif name.endswith("_deg_per_g_s"):
        units = "deg/(g*s)"
    elif name.endswith("_deg_s_per_g"):
        units = "deg*s/g"
    elif name.endswith("_deg_per_g"):
        units = "deg/g"
    elif name.endswith("_g_per_deg"):
        units = "g/deg"
    elif name.endswith(("_deg_s", "_deg_per_s")):
        units = "deg/s"
    elif name.endswith("_deg"):
        units = "deg"
    elif name.endswith("_g_s"):
        units = "g*s"
    elif name.endswith("_g") or name.startswith("actual_specific_force_L_g."):
        units = "g"
    elif name.endswith("_s"):
        units = "s"
    elif name.endswith("_mps"):
        units = "m/s"
    elif name.endswith("_pa"):
        units = "Pa"
    elif name.endswith("_us"):
        units = "us"
    elif name.endswith(("_enabled", "_used", "_active", "_valid", "_count")) or name in (
            "enabled", "realization", "status", "command_limit_kind", "contact_recorded",
            "control.tick_count", "control.png_N") or leaf in ("saturated", "visual_valid",
            "available", "mixed_command_exceeds_limit", "pwm_limited", "pwm_at_limit",
            "held_at_mechanical_limit", "actual_at_mechanical_limit"):
        units = "1"
    if ".feedback_g" in name:
        source = "most recent controller sampled IMU, after feedback sign"
    elif name.startswith("control."):
        source = "most recent controller tick; sample time may differ from t_s"
    elif name.endswith(".command_g") or "axis_command" in name:
        source = "zero-order-held controller command"
    elif name.endswith(".actual_g"):
        source = "physical non-gravity specific force; includes passive drag and contact"
    elif name.endswith(".realized_control_g"):
        source = "ideal actuator control-specific force; excludes passive drag"
    elif "imu_" in name:
        source = "already-sampled controller IMU; no resampling"
    else:
        source = "read-only simulation observation"
    return {"unit": units, "source": source}


class DiagnosticBuffer:
    """Bounded compact samples.  Old replay times can never enter the live buffer."""

    def __init__(self, max_samples=10000):
        if isinstance(max_samples, bool) or int(max_samples) != max_samples or max_samples < 1:
            raise ValueError("max_samples must be a positive integer")
        self.max_samples = int(max_samples)
        self._records = deque()
        self.dropped_samples = 0
        self.rejected_old_samples = 0
        self.generation = 0

    def __len__(self):
        return len(self._records)

    @property
    def truncated_count(self):
        return self.dropped_samples

    def clear(self):
        self._records.clear()
        self.dropped_samples = self.rejected_old_samples = 0
        self.generation += 1

    def append(self, t, obs):
        if not _finite(t) or float(t) < 0:
            raise ValueError("Diagnostic sample time must be finite and nonnegative")
        if not isinstance(obs, Mapping):
            raise TypeError("Diagnostic observation must be a mapping")
        t = float(t)
        if self._records and t < self._records[-1]["t_s"]-1e-12:
            self.rejected_old_samples += 1
            return False
        record = _record(t, obs)
        if self._records and abs(t-self._records[-1]["t_s"]) <= 1e-12:
            self._records[-1] = record
        else:
            if len(self._records) == self.max_samples:
                self._records.popleft()
                self.dropped_samples += 1
            self._records.append(record)
        self.generation += 1
        return True

    def at(self, t):
        if not _finite(t):
            raise ValueError("Diagnostic cutoff must be finite")
        for record in reversed(self._records):
            if record["t_s"] <= float(t)+1e-12:
                return dict(record)
        return None

    def series_until(self, t=None):
        if t is not None and not _finite(t):
            raise ValueError("Diagnostic cutoff must be finite")
        records = list(self._records)
        if t is not None:
            limit = bisect_right([r["t_s"] for r in records], float(t)+1e-12)
            records = records[:limit]
        return [dict(record) for record in records]

    def _window_view(self, t, window_s):
        """Internal read-only plot references; exports still copy full records.

        The last available sample anchors the window, exactly as draw_axis does.
        The viewer and painter run on one Tk thread and never mutate records.
        """
        cutoff = float(t)+1e-12
        count = len(self._records)
        selected, start = [], None
        for record in reversed(self._records):
            timestamp = record["t_s"]
            if timestamp > cutoff:
                count -= 1
                continue
            if start is None:
                start = max(0., timestamp-max(.01, float(window_s)))
            if timestamp < start-1e-12:
                break
            selected.append(record)
        selected.reverse()
        return selected, count

    def export_csv(self, path, until=None):
        records = self.series_until(until)
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        fields = ["t_s"] + sorted({k for record in records for k in record if k != "t_s"})
        metadata = dict(schema_version=SCHEMA_VERSION, rows=len(records),
                        cutoff_s=until, dropped_oldest_samples=self.dropped_samples,
                        rejected_old_time_samples=self.rejected_old_samples,
                        unavailable="empty field; never fabricated zero",
                        nonfinite="retained as nan/inf and counted in invalid_numeric_count",
                        conventions="pitch down/yaw right specific force; body +X roll rate; yaw attitude relative to launch",
                        interpolation="controller and PWM held values are zero-order-held",
                        columns={name: column_metadata(name) for name in fields})
        with path.open("w", encoding="utf-8-sig", newline="") as handle:
            handle.write("# flight_debug_metadata=" + json.dumps(metadata, ensure_ascii=False, allow_nan=False) + "\n")
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(records)
        return path


def _fmt(value, digits=6):
    if value is None:
        return "不可用"
    if isinstance(value, bool):
        return "是" if value else "否"
    if _numeric(value):
        if not math.isfinite(float(value)):
            return str(value)
        return f"{value:.{digits}f}"
    return str(value)


def _curve_specs(axis, mode):
    if mode == "过载":
        return [(f"{axis}.command_g", "请求", "command", True),
                (f"{axis}.actual_g", "物理实际", "actual", False),
                (f"{axis}.feedback_g", "控制反馈", "feedback", True)]
    if mode == "姿态角":
        return [("roll.target_deg" if axis == "roll" else "_unavailable", "目标", "command", True),
                (f"{axis}.attitude_deg", "实际", "actual", False),
                (f"{axis}.imu_attitude_deg", "IMU", "feedback", True)]
    if mode == "角速率":
        return [("roll.rate_command_deg_s" if axis == "roll" else "_unavailable", "目标p" if axis == "roll" else "目标", "command", True),
                (f"{axis}.rate_deg_s", "机体实际", "actual", False),
                (f"{axis}.imu_rate_deg_s", "陀螺反馈", "feedback", True)]
    return [(f"{axis}.axis_command_deg", "混控请求", "command", True),
            (f"{axis}.axis_actual_deg", "实际等效", "actual", False),
            (f"{axis}.axis_held_deg", "PWM保持等效", "feedback", True)]


def _range(values, manual=None, minimum=.001):
    if manual is not None and _finite(manual) and manual > 0:
        return float(manual)
    maximum = max([abs(float(v)) for v in values if _finite(v)] or [0.])
    return max(minimum, maximum*1.15)


class _CanvasPainter:
    def __init__(self, canvas):
        self.canvas = canvas

    def line(self, points, fill, width=1, dash=None):
        if len(points) >= 2:
            options = dict(fill=fill, width=width)
            if dash:
                options["dash"] = dash
            self.canvas.create_line(*[v for xy in points for v in xy], **options)

    def rect(self, box, fill=None, outline=None, width=1):
        self.canvas.create_rectangle(*box, fill=fill or "", outline=outline or "", width=width)

    def ellipse(self, box, fill=None, outline=None, width=1):
        self.canvas.create_oval(*box, fill=fill or "", outline=outline or "", width=width)

    def text(self, xy, value, fill=None, size=12, anchor="nw"):
        self.canvas.create_text(*xy, text=value, fill=fill or PALETTE["text"],
                                font=("Microsoft YaHei UI", size), anchor=anchor)


@lru_cache(maxsize=24)
def _plot_font(size):
    from PIL import ImageFont
    candidates = (Path("C:/Windows/Fonts/msyh.ttc"), Path("C:/Windows/Fonts/simhei.ttf"),
                  Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"))
    source = next((str(p) for p in candidates if p.exists()), None)
    return ImageFont.truetype(source, size) if source else ImageFont.load_default()


class _PillowPainter:
    def __init__(self, image, text_stroke=None):
        from PIL import ImageDraw
        self.draw = ImageDraw.Draw(image)
        self.text_stroke = text_stroke

    def font(self, size):
        return _plot_font(size)

    def line(self, points, fill, width=1, dash=None):
        if len(points) >= 2:
            self.draw.line(points, fill=fill, width=max(1, int(width)))

    def rect(self, box, fill=None, outline=None, width=1):
        self.draw.rectangle(box, fill=fill, outline=outline, width=max(1, int(width)))

    def ellipse(self, box, fill=None, outline=None, width=1):
        self.draw.ellipse(box, fill=fill, outline=outline, width=max(1, int(width)))

    def text(self, xy, value, fill=None, size=12, anchor="nw"):
        font = self.font(size)
        box = self.draw.textbbox((0, 0), str(value), font=font)
        w, h = box[2]-box[0], box[3]-box[1]
        x, y = xy
        if anchor in ("n", "center", "s"):
            x -= w/2
        elif anchor in ("ne", "e", "se"):
            x -= w
        if anchor in ("w", "center", "e"):
            y -= h/2
        elif anchor in ("sw", "s", "se"):
            y -= h
        options = dict(stroke_width=1, stroke_fill=self.text_stroke) if self.text_stroke is not None else {}
        self.draw.text((x-box[0], y-box[1]), str(value), fill=fill or PALETTE["text"], font=font, **options)


def _clip_segment(a, b, box):
    """Liang-Barsky clipping: never fold an out-of-range sample onto the axis."""
    x0, y0, x1, y1 = box
    dx, dy = b[0]-a[0], b[1]-a[1]
    lo, hi = 0., 1.
    for p, q in ((-dx, a[0]-x0), (dx, x1-a[0]), (-dy, a[1]-y0), (dy, y1-a[1])):
        if abs(p) < 1e-20:
            if q < 0:
                return None
        else:
            ratio = q/p
            if p < 0:
                lo = max(lo, ratio)
            else:
                hi = min(hi, ratio)
            if lo > hi:
                return None
    return ((a[0]+lo*dx, a[1]+lo*dy), (a[0]+hi*dx, a[1]+hi*dy))


def _envelope(points):
    """At subpixel density retain each bucket's first, min, max and last sample."""
    bins, output = [], []
    current = None
    for point in points:
        bucket = int(point[0])
        if current is not None and bucket != current:
            indices = sorted({0, len(bins)-1,
                              min(range(len(bins)), key=lambda i: bins[i][1]),
                              max(range(len(bins)), key=lambda i: bins[i][1])})
            output.extend(bins[index] for index in indices)
            bins = []
        current = bucket
        bins.append(point)
    if bins:
        indices = sorted({0, len(bins)-1,
                          min(range(len(bins)), key=lambda i: bins[i][1]),
                          max(range(len(bins)), key=lambda i: bins[i][1])})
        output.extend(bins[index] for index in indices)
    return output


def _curve_values(records, key):
    """One validity pass; missing values break lines and numeric NaNs are counted."""
    values = np.full(len(records), np.nan)
    invalid = 0
    for index, record in enumerate(records):
        value = record.get(key)
        if type(value) in (float, int) or _numeric(value):
            values[index] = value
            invalid += not math.isfinite(value)
    finite = np.isfinite(values)
    peak = float(np.max(np.abs(values[finite]))) if finite.any() else 0.
    return values, finite, invalid, peak


def _straight_points(points):
    """Remove only collinear display vertices; preserve extrema and all corners."""
    if len(points) < 3:
        return points
    array = np.asarray(points, dtype=float)
    delta = np.diff(array, axis=0)
    cross = delta[:-1, 0]*delta[1:, 1]-delta[:-1, 1]*delta[1:, 0]
    keep = np.r_[True, np.abs(cross) > 1e-10, True]
    return [tuple(point) for point in array[keep]]


def draw_axis(painter, width, height, records, axis, mode, visible=None, manual_range=None, window_s=3., dense=False):
    visible = visible or dict(command=True, actual=True, feedback=True)
    painter.rect((0, 0, width, height), fill=PALETTE["surface"])
    narrow = width < 500
    ideal_roll = bool(records and records[-1].get("realization") == "ideal" and axis == "roll")
    title = f"{AXIS_LABELS[axis]} · {mode} [{MODES[mode]}]"
    if ideal_roll and narrow:
        title = f"滚转 · {'固定姿态角' if mode == '姿态角' else mode} [{MODES[mode]}]"
    painter.text((8 if dense else 12, 2 if dense else 8), title, size=9 if dense else (11 if narrow else 13))
    if width < 185 or height < (72 if dense else 185):
        painter.text((12, 32), "图区较小，请拖高面板或滚动查看", size=9, fill=PALETTE["muted"])
        return
    if axis == "roll" and mode == "过载":
        painter.text((width/2, height/2), "滚转没有独立g通道", anchor="center")
        return
    if not records:
        painter.text((width/2, height/2), "当前回放时刻之前没有缓存", anchor="center", fill=PALETTE["muted"])
        return
    t1 = records[-1]["t_s"]
    t0 = max(0., t1-max(.01, window_s))
    records = [r for r in records if r["t_s"] >= t0-1e-12]
    specs = _curve_specs(axis, mode)
    prepared = {key: _curve_values(records, key) for key, _, color, _ in specs if visible.get(color, True)}
    minimum = .001 if mode == "过载" else .1
    peak = max((item[3] for item in prepared.values()), default=0.)
    limit = float(manual_range) if _finite(manual_range) and manual_range > 0 else max(minimum, peak*1.15)
    if dense:
        warnings = []
        if peak > limit:
            warnings.append("↑超量程")
        if any(item[2] for item in prepared.values()):
            warnings.append("?非有限")
        if warnings:
            painter.text((width-8, 2), " ".join(warnings), anchor="ne", size=8, fill=PALETTE["warning"])
    if dense:
        left, top, right, bottom = 54., 33., width-15., height-18.
    else:
        left, top, right, bottom = (66. if narrow else 76.), (83. if narrow else 58.), width-23., height-31.
    plotbox = (left, top, right, bottom)
    scale_x = (right-left)/max(.01, t1-t0)
    scale_y = (bottom-top)/(2*limit)
    for ratio in ((-1., 0., 1.) if dense else (-1., -.5, 0., .5, 1.)):
        y = (top+bottom)/2-ratio*limit*scale_y
        painter.line(((left, y), (right, y)), PALETTE["grid"] if ratio else "#A9BDCD")
        painter.text((left-5 if dense else left-7, y), f"{ratio*limit:.3g}", fill=PALETTE["muted"], anchor="e", size=8 if dense else (9 if narrow else 10))
    tick_count = 2 if dense and height < 105 else (3 if dense else 5)
    for index in range(tick_count):
        ratio = index/(tick_count-1)
        x = left+(right-left)*ratio
        painter.line(((x, top), (x, bottom)), PALETTE["grid"])
        painter.text((x, bottom+(3 if dense else 7)), f"{t0+(t1-t0)*ratio:.2f}s" if narrow else f"{t0+(t1-t0)*ratio:.3f}s",
                     anchor="n", size=8 if dense else (9 if narrow else 10), fill=PALETTE["muted"])
    for idx, (key, label, color, held) in enumerate(specs):
        if not visible.get(color, True):
            continue
        values, finite, invalid_count, _peak = prepared[key]
        present = bool(finite.any())
        value = records[-1].get(key)
        overflow_count = int(np.count_nonzero(finite & (np.abs(values) > limit)))
        text = f"{label}: {_fmt(value) if present else '无数据/设定值'}"
        if dense:
            short = ("请", "实", "保" if mode == "等效舵量" else "反")[idx]
            text = short+((f"{value:.6g}" if _finite(value) else _fmt(value)) if present else "NA")
            if overflow_count:
                text += "↑"
            if invalid_count:
                text += "?"
        elif narrow and (overflow_count or invalid_count):
            text += f" 超限{overflow_count}/非有限{invalid_count}"
        painter.text(((8+idx*(width-16)/3) if dense else (12 if narrow else 12+idx*(width-20)/3),
                      17 if dense else (29+idx*16 if narrow else 32)),
                     text, fill=PALETTE[color], size=8 if dense else (9 if narrow else 10))
        times = np.fromiter((record["t_s"] for record in records), dtype=float, count=len(records))
        xs = left+(times-t0)*scale_x
        ys = (top+bottom)/2-values*scale_y
        boundaries = np.flatnonzero(np.diff(np.r_[False, finite, False]))
        runs = [list(zip(xs[start:end], ys[start:end])) for start, end in zip(boundaries[::2], boundaries[1::2])]
        overflow, invalid = overflow_count, invalid_count
        for run in runs:
            if not held:
                run = _straight_points(run)
            if len(run) > max(8, 2*(right-left)):
                run = _envelope(run)
            if len(run) == 1:
                x, y = run[0]
                if top <= y <= bottom:
                    painter.ellipse((x-2, y-2, x+2, y+2), fill=PALETTE[color])
            polyline = []
            for point in run:
                if held and polyline:
                    polyline.append((point[0], polyline[-1][1]))
                polyline.append(point)
            polyline = _straight_points(polyline)
            if not overflow:
                painter.line(polyline, PALETTE[color], 2)
                continue
            # Group contiguous clipped segments to reduce Tk canvas objects.
            connected = []
            for a, b in zip(polyline, polyline[1:]):
                segment = _clip_segment(a, b, plotbox)
                if segment is None:
                    if connected:
                        painter.line(connected, PALETTE[color], 2)
                        connected = []
                elif connected and segment[0] == connected[-1]:
                    connected.append(segment[1])
                else:
                    if connected:
                        painter.line(connected, PALETTE[color], 2)
                    connected = list(segment)
            if connected:
                painter.line(connected, PALETTE[color], 2)
        if (overflow or invalid) and not narrow and not dense:
            painter.text((right-idx*145, top+3), f"{label} 超量程{overflow} 非有限{invalid}",
                         anchor="ne", fill=PALETTE["warning"], size=9)
    previous_bad = None
    for record in records:
        bad = record.get("status") in ("initial", "stale", "awaiting_visual")
        if bad and previous_bad is not True:
            x = left+(record["t_s"]-t0)*scale_x
            painter.line(((x, top), (x, bottom)), "#D5A879", 1, (2, 4))
        previous_bad = bad
    contact_times = {r.get("contact_time_s") for r in records if _finite(r.get("contact_time_s"))}
    for t in contact_times:
        if t0 <= t <= t1:
            x = left+(t-t0)*scale_x
            painter.line(((x, top), (x, bottom)), PALETTE["warning"], 1, (3, 3))
            if not dense:
                painter.text((x+3, bottom-3), "接触", fill=PALETTE["warning"], anchor="sw", size=9)
    if ideal_roll and not narrow:
        painter.text((right, height-5), "简化后端：欧拉滚转固定，无滚转AP/舵机", anchor="se", size=9, fill=PALETTE["muted"])


def draw_g_vector(painter, width, height, record, visible=None, manual_range=None):
    visible = visible or dict(command=True, actual=True, feedback=True)
    painter.rect((0, 0, width, height), fill=PALETTE["surface"])
    narrow = width < 400
    painter.text((12, 8), "g矢量 · 偏航右 / 俯仰下" if narrow else "有符号g矢量 · 横向偏航 / 纵向俯仰", size=10 if narrow else 12)
    if width < 185 or height < 185:
        painter.text((12, 32), "图区较小，请拖高面板或滚动查看", size=9, fill=PALETTE["muted"])
        return
    if not record:
        painter.text((width/2, height/2), "暂无数据", anchor="center")
        return
    values = [record.get(f"{axis}.{kind}_g") for axis in ("pitch", "yaw")
              for kind in ("command", "actual", "feedback") if visible.get(kind, True)]
    limit = _range(values, manual_range, .001)
    plot_height = max(20., height-(136 if narrow else 100))
    radius = min(width*.40, plot_height*.44 if narrow else plot_height*.47)
    cx, cy = width/2, (35 if narrow else 42)+plot_height/2
    for ratio in (.25, .5, .75, 1.):
        r = ratio*radius
        painter.ellipse((cx-r, cy-r, cx+r, cy+r), outline=PALETTE["grid"])
    painter.line(((cx-radius, cy), (cx+radius, cy)), "#9CAFC0")
    painter.line(((cx, cy-radius), (cx, cy+radius)), "#9CAFC0")
    painter.text((cx+radius, cy-5), "+偏航右", anchor="se", size=9)
    painter.text((cx+5, cy+radius+4), "+俯仰下", size=9)
    painter.text((12, height-(82 if narrow else 55)), f"量程 ±{limit:.4g}g · 圈线仅为量程" if narrow else f"显示半量程 ±{limit:.5g} g（圈线仅为量程）", size=9 if narrow else 10, fill=PALETTE["muted"])
    request_limit = record.get("command_limit_g")
    kind = record.get("command_limit_kind")
    if _finite(request_limit) and request_limit > 0 and kind in ("per_axis", "combined_norm"):
        displayed = min(float(request_limit)/limit, 1.)*radius
        box = (cx-displayed, cy-displayed, cx+displayed, cy+displayed)
        if request_limit <= limit:
            if kind == "per_axis":
                painter.rect(box, outline="#788BA0")
            else:
                painter.ellipse(box, outline="#788BA0", width=2)
        domain = "方形请求域" if kind == "per_axis" else "合成圆形限幅"
        suffix = "（界外）" if request_limit > limit else ""
        painter.text((12, height-(65 if narrow else 37)), f"{domain}: {request_limit:g}g {suffix}", size=9, fill=PALETTE["muted"])
    for idx, (field, label) in enumerate((("command", "请求"), ("actual", "物理"), ("feedback", "反馈"))):
        if not visible.get(field, True):
            continue
        pitch, yaw = record.get(f"pitch.{field}_g"), record.get(f"yaw.{field}_g")
        if _finite(pitch) and _finite(yaw):
            outside = max(abs(pitch), abs(yaw)) > limit
            scale = max(1., max(abs(pitch), abs(yaw))/limit)
            x, y = cx+yaw/limit/scale*radius, cy+pitch/limit/scale*radius
            painter.line(((cx, cy), (x, y)), PALETTE[field], 2)
            painter.ellipse((x-3, y-3, x+3, y+3), fill=PALETTE[field])
            if outside:
                painter.text((x, y), "△" if narrow else "△超量程", anchor="s", fill=PALETTE["warning"], size=9)
            norm = math.hypot(pitch, yaw)
            text = f"{label} {norm:.6g}g" + (" 超量程" if outside and narrow else "")
        else:
            text = f"{label} 不可用"
        # On compact cards each vector gets its own readout row; six-digit g
        # values must not overlap neighbours or be truncated by the card edge.
        if narrow:
            painter.text((12, height-45+idx*14), text, fill=PALETTE[field], size=9)
        else:
            painter.text((12+idx*(width-20)/3, height-19), text, fill=PALETTE[field], size=10)


def _state_lines(record):
    if not record:
        return [("暂无诊断样本", False)]
    r = record
    lines = [(f"后端 {r.get('realization')} · 控制 {_fmt(r.get('enabled'))} · {r.get('status')}", False),
             ("四舵  请求° / PWM保持° / 实际° / PWM µs", False)]
    if r.get("realization") == "ideal":
        lines.append(("简化后端未使用舵机、自驾仪及滚转控制", False))
    else:
        for i in range(4):
            vals = " / ".join(_fmt(r.get(f"fin{i}.{key}"), 2) for key in ("command_deg", "held_deg", "actual_deg", "pwm_us"))
            mixed = r.get(f"fin{i}.mixed_command_exceeds_limit")
            pwm = r.get(f"fin{i}.pwm_limited")
            pwm_at_limit = r.get(f"fin{i}.pwm_at_limit")
            held = r.get(f"fin{i}.held_at_mechanical_limit")
            actual = r.get(f"fin{i}.actual_at_mechanical_limit")
            request_flags = f"当前混控越限{_fmt(mixed)} 当前请求PWM限幅{_fmt(pwm)}"
            held_flags = f"保持PWM边界{_fmt(pwm_at_limit)} 保持机械限位{_fmt(held)} 实际机械限位{_fmt(actual)}"
            lines.extend(((f"CH{i}: {vals}", False),
                          ("  "+request_flags, any(v is True for v in (mixed, pwm))),
                          ("  "+held_flags, any(v is True for v in (pwm_at_limit, held, actual)))))
        lines.append(("请求诊断t="+_fmt(r.get("actuator_diagnostics.request_time_s"))+"s · 保持PWM采样t="+
                      _fmt(r.get("actuator_diagnostics.held_pwm_time_s"))+"s", False))
    lines.append(("自驾仪未使用（以下为兼容字段，不能解释为执行环）" if r.get("realization") == "ideal"
                  else "自驾仪内部（控制tick采样值）", False))
    for axis in ("pitch", "yaw"):
        prefix = f"control.autopilot_{axis}."
        lines.extend(((f"{AXIS_LABELS[axis]} α̂={_fmt(r.get(prefix+'alpha_hat_deg'))}°  I={_fmt(r.get(prefix+'integral_g_s'))}g·s", False),
                      (f"  PNG={_fmt(r.get('control.a_png_'+axis+'_g'))}  瞄准={_fmt(r.get('control.a_aim_'+axis+'_g'))}g", False),
                      (f"  反馈原值={_fmt(r.get('control.a_measured_raw_'+axis+'_g'))}  符号后={_fmt(r.get(axis+'.feedback_g'))}g", False),
                      (f"  过载误差={_fmt(r.get('control.a_error_'+axis+'_g'))}g  视线速率={_fmt(r.get('control.q_dot_'+axis+'_deg_s'))}°/s", False)))
        cfg = "control.effective_axis_config."+axis+"."
        text = "  " + "  ".join(label+"="+_fmt(r.get(cfg+key), 3) for key, label in
                                (("png_N", "N"), ("k_rate", "Kr"), ("k_alpha", "Ka"), ("k_int", "Ki")))
        lines.append((text, False))
        diag = "control.axis_diagnostics."+axis+"."
        lines.extend((("  轴舵原值="+_fmt(r.get(diag+"unlimited_deg"))+"° 限幅后="+
                       _fmt(r.get(diag+"command_deg"))+"° 上限="+_fmt(r.get(diag+"limit_deg"))+"° 饱和="+
                       _fmt(r.get(diag+"saturated")), r.get(diag+"saturated") is True),
                      ("  速率项="+_fmt(r.get(diag+"rate_term_deg"))+"° α项="+
                       _fmt(r.get(diag+"alpha_term_deg"))+"° 积分项="+_fmt(r.get(diag+"integral_term_deg"))+"°", False)))
    lines.extend(((f"滚转模式 {r.get('control.roll_mode', '不可用')} · 激活 {_fmt(r.get('control.roll_active'))}", False),
                  (f"滚转目标 {_fmt(r.get('roll.target_deg'))}° · p指令 {_fmt(r.get('roll.rate_command_deg_s'))}°/s", False),
                  ("滚转轴舵原值="+_fmt(r.get("control.axis_diagnostics.roll.unlimited_deg"))+"° 限幅后="+
                   _fmt(r.get("control.axis_diagnostics.roll.command_deg"))+"° 饱和="+
                   _fmt(r.get("control.axis_diagnostics.roll.saturated")), r.get("control.axis_diagnostics.roll.saturated") is True),
                  ("视觉 / 时钟 / 载荷", False),
                  (f"t={_fmt(r.get('t_s'),6)}s  实际量时刻={_fmt(r.get('actual_time_s'),6)}s", False),
                  (f"控制t={_fmt(r.get('control.control_time_s'),6)}s  tick={_fmt(r.get('control.tick_count'),0)}", False),
                  (f"视觉有效 {_fmt(r.get('control.visual_valid'))} · 年龄 {_fmt(r.get('visual_age_s'),4)}s", False),
                  (f"速度 {_fmt(r.get('speed_mps'),4)}m/s · 动压 {_fmt(r.get('dynamic_pressure_pa'),3)}Pa", False),
                  (f"机体系比力 xyz= {_fmt(r.get('actual_specific_force_L_g.x'))}, {_fmt(r.get('actual_specific_force_L_g.y'))}, {_fmt(r.get('actual_specific_force_L_g.z'))}g", False),
                  (f"接触记录 {_fmt(r.get('contact_recorded'))} · 非有限数 {_fmt(r.get('invalid_numeric_count'),0)}", bool(r.get("contact_recorded") or r.get("invalid_numeric_count"))),
                  ("物理比力包含被动气动及接触；不是惯性加速度", False),
                  ("姿态偏航相对发射方向；角速率为机体系分量", False),
                  ("请求域/执行模型参数均不代表已验证实物能力", False)))
    return lines


def draw_states(painter, width, height, record):
    painter.rect((0, 0, width, height), fill=PALETTE["surface"])
    painter.text((12, 8), "执行器与内部状态", size=13)
    for index, (text, warning) in enumerate(_state_lines(record)):
        painter.text((12, 36+index*21), text, size=10,
                     fill=PALETTE["warning"] if warning else PALETTE["text"])


def render_g_hud(image, record, *, size=220, visible=None, manual_range=None, corner="bottom_right"):
    """Return a display-only RGB copy with a transparent signed-g corner HUD.

    Call this *after* resizing a viewport for display.  The input camera pixels
    and every diagnostic value remain unchanged.  No simulation object is read.
    """
    from PIL import Image, ImageDraw
    if corner not in ("bottom_right", "bottom_left", "top_right", "top_left"):
        raise ValueError("Unknown HUD corner")
    if image.width < 1 or image.height < 1:
        return image.convert("RGB").copy()
    visible = visible or dict(command=True, actual=True, feedback=True)
    available = min(image.width-16, image.height-16)
    if available < 100:
        return image.convert("RGB").copy()
    requested = int(size) if _finite(size) else 220
    # Keep the HUD readable in a wide, short viewport; the width bound leaves
    # the central camera aiming area clear on a native 320x240 image.
    side = min(available, 240, max(140, min(requested, round(image.height*.54),
                                         round(image.width*.44))))
    hud = Image.new("RGBA", (side, side), (0, 0, 0, 0))
    draw = ImageDraw.Draw(hud)
    painter = _PillowPainter(hud, text_stroke=(0, 0, 0, 185))
    ink = (237, 247, 255, 238)
    muted = (204, 225, 238, 210)
    grid = (211, 231, 243, 80)
    colors = {"command": (91, 207, 255, 255), "actual": (109, 242, 188, 255),
              "feedback": (255, 199, 104, 255)}
    # A weak tint supports both light and dark fields; this is not a solid card.
    draw.rounded_rectangle((1, 1, side-2, side-2), radius=9,
                           fill=(11, 31, 47, 68), outline=(219, 238, 249, 72))
    font_size = 9 if side < 180 else 11
    painter.text((8, 7), "g · 右+偏航 / 下+俯仰", fill=ink, size=font_size)
    if not record:
        painter.text((side/2, side/2), "暂无诊断", fill=muted, size=font_size, anchor="center")
    else:
        values = [record.get(f"{axis}.{field}_g") for axis in ("pitch", "yaw")
                  for field in ("command", "actual", "feedback") if visible.get(field, True)]
        limit = _range(values, manual_range, .001)
        cx, cy, radius = side*.50, side*.43, side*.24
        for ratio in (.5, 1.):
            r = radius*ratio
            draw.ellipse((cx-r, cy-r, cx+r, cy+r), outline=(0, 0, 0, 95), width=3)
            draw.ellipse((cx-r, cy-r, cx+r, cy+r), outline=grid)
        draw.line((cx-radius, cy, cx+radius, cy), fill=(0, 0, 0, 95), width=3)
        draw.line((cx, cy-radius, cx, cy+radius), fill=(0, 0, 0, 95), width=3)
        draw.line((cx-radius, cy, cx+radius, cy), fill=grid)
        draw.line((cx, cy-radius, cx, cy+radius), fill=grid)
        request_limit, kind = record.get("command_limit_g"), record.get("command_limit_kind")
        domain = f"量程±{limit:.3g}g"
        if _finite(request_limit) and request_limit > 0 and kind in ("per_axis", "combined_norm"):
            label = "方请" if kind == "per_axis" else "圆限"
            domain += f" {label}{request_limit:.3g}g"
            if request_limit <= limit:
                r = request_limit/limit*radius
                box = (cx-r, cy-r, cx+r, cy+r)
                if kind == "per_axis":
                    draw.rectangle(box, outline=(0, 0, 0, 120), width=3)
                    draw.rectangle(box, outline=(214, 231, 243, 140))
                else:
                    draw.ellipse(box, outline=(0, 0, 0, 120), width=4)
                    draw.ellipse(box, outline=(214, 231, 243, 140), width=2)
            else:
                domain += "外"
        painter.text((8, 23 if side < 180 else 25), domain, fill=muted, size=8 if side < 180 else 9)
        for index, (field, label) in enumerate((("command", "请求"), ("actual", "实际"), ("feedback", "反馈"))):
            if not visible.get(field, True):
                continue
            pitch, yaw = record.get(f"pitch.{field}_g"), record.get(f"yaw.{field}_g")
            text = label+" NA"
            if _finite(pitch) and _finite(yaw):
                outside = max(abs(pitch), abs(yaw)) > limit
                factor = max(1., max(abs(pitch), abs(yaw))/limit)
                x, y = cx+yaw/limit/factor*radius, cy+pitch/limit/factor*radius
                draw.line((cx, cy, x, y), fill=(0, 0, 0, 150), width=4)
                draw.line((cx, cy, x, y), fill=colors[field], width=2)
                draw.ellipse((x-3, y-3, x+3, y+3), fill=(0, 0, 0, 180))
                draw.ellipse((x-2, y-2, x+2, y+2), fill=colors[field])
                text = f"{label} {math.hypot(pitch,yaw):.6g}g" + (" 超限" if outside else "")
            painter.text((8, side-44+index*14), text, fill=colors[field], size=font_size)
    x = image.width-side-8 if corner.endswith("right") else 8
    y = image.height-side-8 if corner.startswith("bottom") else 8
    viewport = image.convert("RGB").copy()
    patch = viewport.crop((x, y, x+side, y+side)).convert("RGBA")
    patch.alpha_composite(hud)
    viewport.paste(patch.convert("RGB"), (x, y))
    return viewport


class FlightDebugWindow:
    """Embeddable or independent monitor; its cache lives outside Tk widgets."""

    def __init__(self, root, buffer, displayed_time_provider, *, parent=None, compact=False, stacked=False,
                 on_tune_axis=None):
        self.root, self.buffer = root, buffer
        self.parent, self.compact = parent, bool(compact)
        self.stacked = bool(stacked)
        self.displayed_time_provider = displayed_time_provider
        self.window = None
        self._shown = False
        self._last_key = None
        self._displayed_t = 0.
        self._canvases = {}
        self.on_tune_axis = on_tune_axis
        self._tuning_hosts, self._tune_buttons = {}, {}

    def _build(self):
        import tkinter as tk
        from tkinter import ttk
        if self.parent is not None:
            self.window = ttk.Frame(self.parent)
            self.window.pack(fill="both", expand=True)
        else:
            self.window = tk.Toplevel(self.root)
            self.window.title("三轴飞控调试 · 只读监视器")
            self.window.geometry("1440x930")
            self.window.minsize(700, 450)
            self.window.configure(background=PALETTE["background"])
            self.window.protocol("WM_DELETE_WINDOW", self.hide)
        if self.stacked:
            self._build_stacked()
            return
        if self.compact:
            self._build_compact()
            return
        outer = ttk.Frame(self.window)
        outer.pack(fill="both", expand=True)
        scroll = tk.Canvas(outer, background=PALETTE["background"], highlightthickness=0)
        ybar = ttk.Scrollbar(outer, orient="vertical", command=scroll.yview)
        xbar = ttk.Scrollbar(outer, orient="horizontal", command=scroll.xview)
        scroll.configure(yscrollcommand=ybar.set, xscrollcommand=xbar.set)
        ybar.pack(side="right", fill="y")
        xbar.pack(side="bottom", fill="x")
        scroll.pack(fill="both", expand=True)
        body = ttk.Frame(scroll, padding=12)
        item = scroll.create_window(0, 0, window=body, anchor="nw")
        body.bind("<Configure>", lambda _: scroll.configure(scrollregion=scroll.bbox("all")))
        scroll.bind("<Configure>", lambda e: scroll.itemconfigure(item, width=max(1190, e.width)))
        def wheel(event):
            scroll.yview_scroll(int(-event.delta/120), "units")
        # Bind to the window's bindtag only; never steal the main viewer's wheel.
        self.window.bind("<MouseWheel>", wheel)
        toolbar = ttk.Frame(body)
        toolbar.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        self._status = tk.StringVar(value="等待只读诊断")
        ttk.Label(toolbar, textvariable=self._status).pack(side="left", padx=(0, 12))
        ttk.Button(toolbar, text="导出CSV", command=self._export_csv).pack(side="right", padx=3)
        self._visible = {key: tk.BooleanVar(value=True) for key in ("command", "actual", "feedback")}
        for key, label in (("command", "请求/目标"), ("actual", "物理实际"), ("feedback", "反馈/保持")):
            ttk.Checkbutton(toolbar, text=label, variable=self._visible[key], command=self._invalidate).pack(side="left", padx=3)
        controls = ttk.Frame(body)
        controls.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        ttk.Label(controls, text="显示窗口(s)").pack(side="left")
        self._time_window = tk.StringVar(value="3")
        ttk.Entry(controls, textvariable=self._time_window, width=7).pack(side="left", padx=5)
        ttk.Label(controls, text="量程填写正半量程；自动量程保留尖峰。无设定值显示不可用。").pack(side="left", padx=10)
        ttk.Button(controls, text="应用显示设置", command=self._invalidate).pack(side="left")
        self._modes, self._auto, self._manual = {}, {}, {}
        left = ttk.Frame(body)
        left.grid(row=2, column=0, sticky="nsew", padx=(0, 10))
        right = ttk.Frame(body)
        right.grid(row=2, column=1, sticky="nsew")
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=2)
        for axis in AXIS_LABELS:
            card = ttk.LabelFrame(left, text=AXIS_LABELS[axis], padding=5)
            card.pack(fill="both", expand=True, pady=(0, 8))
            row = ttk.Frame(card)
            row.pack(fill="x")
            self._modes[axis] = tk.StringVar(value="姿态角" if axis == "roll" else "过载")
            options = ["姿态角", "角速率", "等效舵量"] if axis == "roll" else list(MODES)
            combo = ttk.Combobox(row, textvariable=self._modes[axis], values=options, state="readonly", width=11)
            combo.pack(side="left", padx=3)
            combo.bind("<<ComboboxSelected>>", self._invalidate)
            self._auto[axis] = tk.BooleanVar(value=True)
            ttk.Checkbutton(row, text="自动量程", variable=self._auto[axis], command=self._invalidate).pack(side="left", padx=5)
            self._manual[axis] = tk.StringVar(value="1")
            ttk.Entry(row, textvariable=self._manual[axis], width=8).pack(side="left", padx=3)
            ttk.Label(row, text="±（当前单位）").pack(side="left")
            canvas = tk.Canvas(card, width=690, height=240, highlightthickness=0, bg=PALETTE["surface"])
            canvas.pack(fill="both", expand=True)
            canvas.bind("<Configure>", self._invalidate)
            self._canvases[axis] = canvas
        gcard = ttk.LabelFrame(right, text="g矢量", padding=5)
        gcard.pack(fill="x", pady=(0, 8))
        row = ttk.Frame(gcard)
        row.pack(fill="x")
        self._auto["g"] = tk.BooleanVar(value=True)
        self._manual["g"] = tk.StringVar(value="1")
        ttk.Checkbutton(row, text="自动量程", variable=self._auto["g"], command=self._invalidate).pack(side="left")
        ttk.Entry(row, textvariable=self._manual["g"], width=8).pack(side="left", padx=5)
        ttk.Label(row, text="±g（请求域非实物能力）").pack(side="left")
        gcanvas = tk.Canvas(gcard, width=460, height=310, highlightthickness=0, bg=PALETTE["surface"])
        gcanvas.pack(fill="x")
        gcanvas.bind("<Configure>", self._invalidate)
        self._canvases["g"] = gcanvas
        statecanvas = tk.Canvas(right, width=460, height=1100, highlightthickness=0, bg=PALETTE["surface"])
        statecanvas.pack(fill="both", expand=True)
        statecanvas.bind("<Configure>", self._invalidate)
        self._canvases["states"] = statecanvas
        ttk.Label(body, text="缓存按真实宏步时刻记录；回放/导出仅取当前游标之前。蓝指令为零阶保持；虚线标记视觉失效/接触。").grid(
            row=3, column=0, columnspan=2, sticky="w", pady=7)

    def _build_stacked(self):
        """Three equal-height plots in the large viewport's narrow side rail."""
        import tkinter as tk
        from tkinter import ttk
        outer = ttk.Frame(self.window, padding=(5, 3))
        outer.pack(fill="both", expand=True)
        toolbar = ttk.Frame(outer)
        toolbar.pack(fill="x", pady=(0, 2))
        self._status = tk.StringVar(value="等待只读诊断")
        ttk.Label(toolbar, textvariable=self._status).pack(side="left")
        ttk.Button(toolbar, text="CSV", width=5, command=self._export_csv).pack(side="right", padx=1)
        options = ttk.Frame(outer)
        options.pack(fill="x", pady=(0, 3))
        self._time_window = tk.StringVar(value="3")
        ttk.Label(options, text="窗口(s)").pack(side="left")
        entry = ttk.Entry(options, textvariable=self._time_window, width=4)
        entry.pack(side="left", padx=(2, 6))
        entry.bind("<Return>", self._invalidate)
        entry.bind("<FocusOut>", self._invalidate)
        self._visible = {key: tk.BooleanVar(value=True) for key in ("command", "actual", "feedback")}
        for key, label in (("command", "请求"), ("actual", "实际"), ("feedback", "反馈")):
            ttk.Checkbutton(options, text=label, variable=self._visible[key], command=self._invalidate).pack(side="left")
        self.notebook = ttk.Notebook(outer)
        self.notebook.pack(fill="both", expand=True)
        plots = ttk.Frame(self.notebook)
        detail = ttk.Frame(self.notebook)
        self.notebook.add(plots, text="三轴波形")
        self.notebook.add(detail, text="执行器 / 内部状态")
        self.notebook.bind("<<NotebookTabChanged>>", self._invalidate)
        scroll = tk.Canvas(plots, bg=PALETTE["background"], highlightthickness=0)
        ybar = ttk.Scrollbar(plots, orient="vertical", command=scroll.yview)
        scroll.configure(yscrollcommand=ybar.set)
        ybar.pack(side="right", fill="y")
        scroll.pack(fill="both", expand=True)
        column = ttk.Frame(scroll)
        item = scroll.create_window(0, 0, window=column, anchor="nw")
        def fit(event=None):
            # All three waveforms fit together at normal laptop heights.
            # Only exceptionally short panes need to scroll the 330px content.
            minimum = 0
            for index, axis in enumerate(AXIS_LABELS):
                host = self._tuning_hosts.get(axis)
                extra = host.winfo_reqheight()+4 if host is not None and host.winfo_manager() else 0
                column.rowconfigure(index, minsize=110+extra)
                minimum += 110+extra
            scroll.itemconfigure(item, width=max(1, scroll.winfo_width()),
                                 height=max(minimum, scroll.winfo_height()))
        self._fit_stacked = fit
        scroll.bind("<Configure>", fit)
        column.bind("<Configure>", lambda _: scroll.configure(scrollregion=scroll.bbox("all")))
        scroll.bind("<MouseWheel>", lambda event: scroll.yview_scroll(int(-event.delta/120), "units"))
        self._modes, self._auto, self._manual = {}, {}, {}
        column.columnconfigure(0, weight=1)
        for index, axis in enumerate(AXIS_LABELS):
            column.rowconfigure(index, weight=1, minsize=110)
            card = ttk.Frame(column, padding=(3, 3))
            card.grid(row=index, column=0, sticky="nsew")
            controls = ttk.Frame(card)
            controls.pack(fill="x", pady=(0, 2))
            ttk.Label(controls, text=AXIS_LABELS[axis]).pack(side="left", padx=(0, 4))
            self._modes[axis] = tk.StringVar(value="姿态角" if axis == "roll" else "过载")
            modes = ["姿态角", "角速率", "等效舵量"] if axis == "roll" else list(MODES)
            combo = ttk.Combobox(controls, textvariable=self._modes[axis], values=modes,
                                 state="readonly", width=6)
            combo.pack(side="left", padx=(0, 3))
            combo.bind("<<ComboboxSelected>>", self._invalidate)
            self._auto[axis] = tk.BooleanVar(value=True)
            ttk.Checkbutton(controls, text="自动", variable=self._auto[axis], command=self._invalidate).pack(side="left")
            ttk.Label(controls, text="±").pack(side="left")
            self._manual[axis] = tk.StringVar(value="1")
            value = ttk.Entry(controls, textvariable=self._manual[axis], width=4)
            value.pack(side="left", padx=1)
            value.bind("<Return>", self._invalidate)
            value.bind("<FocusOut>", self._invalidate)
            if self.on_tune_axis is not None:
                button = ttk.Button(controls, text="调参", width=3, style="Small.TButton",
                                    command=lambda name=axis: self.on_tune_axis(name))
                button.pack(side="right", padx=(3, 0))
                self._tune_buttons[axis] = button
                self._tuning_hosts[axis] = ttk.Frame(card)
            canvas = tk.Canvas(card, width=330, height=105, bg=PALETTE["surface"], highlightthickness=0)
            canvas.pack(fill="both", expand=True)
            canvas.bind("<Configure>", self._invalidate)
            canvas.bind("<MouseWheel>", lambda event: scroll.yview_scroll(int(-event.delta/120), "units"))
            self._canvases[axis] = canvas
        # The g HUD belongs to the viewport display layer.  Retain a default
        # range in settings for API/export compatibility, without a fourth card.
        self._auto["g"] = tk.BooleanVar(value=True)
        self._manual["g"] = tk.StringVar(value="1")
        states = tk.Canvas(detail, bg=PALETTE["surface"], highlightthickness=0)
        vertical = ttk.Scrollbar(detail, orient="vertical", command=states.yview)
        horizontal = ttk.Scrollbar(detail, orient="horizontal", command=states.xview)
        states.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set,
                         scrollregion=(0, 0, 1250, 1100))
        vertical.pack(side="right", fill="y")
        horizontal.pack(side="bottom", fill="x")
        states.pack(fill="both", expand=True)
        states.bind("<Configure>", self._invalidate)
        states.bind("<MouseWheel>", lambda event: states.yview_scroll(int(-event.delta/120), "units"))
        self._canvases["states"] = states

    def _build_compact(self):
        """Three plots and a signed g view share the main viewer's lower pane."""
        import tkinter as tk
        from tkinter import ttk
        outer = ttk.Frame(self.window, padding=(6, 3))
        outer.pack(fill="both", expand=True)
        toolbar = ttk.Frame(outer)
        toolbar.pack(fill="x", pady=(0, 3))
        self._status = tk.StringVar(value="等待只读诊断")
        ttk.Label(toolbar, textvariable=self._status).pack(side="left", padx=(0, 8))
        ttk.Button(toolbar, text="CSV", width=6, command=self._export_csv).pack(side="right", padx=2)
        self._visible = {key: tk.BooleanVar(value=True) for key in ("command", "actual", "feedback")}
        for key, label in (("feedback", "反馈"), ("actual", "实际"), ("command", "请求")):
            ttk.Checkbutton(toolbar, text=label, variable=self._visible[key], command=self._invalidate).pack(side="right")
        self._time_window = tk.StringVar(value="3")
        time_entry = ttk.Entry(toolbar, textvariable=self._time_window, width=4)
        time_entry.pack(side="right", padx=3)
        time_entry.bind("<Return>", self._invalidate)
        ttk.Label(toolbar, text="时间窗(s)").pack(side="right")
        self.notebook = ttk.Notebook(outer)
        self.notebook.pack(fill="both", expand=True)
        plots = ttk.Frame(self.notebook)
        detail = ttk.Frame(self.notebook)
        self.notebook.add(plots, text="三轴曲线与g矢量")
        self.notebook.add(detail, text="四舵 / 自驾仪 / 视觉与时钟")
        self.notebook.bind("<<NotebookTabChanged>>", self._invalidate)
        scroll = tk.Canvas(plots, background=PALETTE["background"], highlightthickness=0)
        xbar = ttk.Scrollbar(plots, orient="horizontal", command=scroll.xview)
        ybar_plots = ttk.Scrollbar(plots, orient="vertical", command=scroll.yview)
        scroll.configure(xscrollcommand=xbar.set, yscrollcommand=ybar_plots.set)
        xbar.pack(side="bottom", fill="x")
        ybar_plots.pack(side="right", fill="y")
        scroll.pack(fill="both", expand=True)
        row = ttk.Frame(scroll)
        item = scroll.create_window(0, 0, window=row, anchor="nw")
        def fit(event):
            # Keep controls plus a legible plot tall enough.  The vertical
            # scrollbar exposes the entire row when the main pane is smaller.
            scroll.itemconfigure(item, width=max(1060, event.width), height=max(270, event.height))
        scroll.bind("<Configure>", fit)
        row.bind("<Configure>", lambda _: scroll.configure(scrollregion=scroll.bbox("all")))
        scroll.bind("<MouseWheel>", lambda event: scroll.yview_scroll(int(-event.delta/120), "units"))
        self._modes, self._auto, self._manual = {}, {}, {}
        for index, axis in enumerate(AXIS_LABELS):
            row.columnconfigure(index, weight=1, uniform="debug_axis")
            card = ttk.Frame(row, padding=(3, 3))
            card.grid(row=0, column=index, sticky="nsew")
            controls = ttk.Frame(card)
            controls.pack(fill="x")
            ttk.Label(controls, text=AXIS_LABELS[axis]).pack(side="left", padx=(0, 4))
            self._modes[axis] = tk.StringVar(value="姿态角" if axis == "roll" else "过载")
            options = ["姿态角", "角速率", "等效舵量"] if axis == "roll" else list(MODES)
            combo = ttk.Combobox(controls, textvariable=self._modes[axis], values=options,
                                 state="readonly", width=8)
            combo.pack(side="left")
            combo.bind("<<ComboboxSelected>>", self._invalidate)
            scale = ttk.Frame(card)
            scale.pack(fill="x", pady=1)
            self._auto[axis] = tk.BooleanVar(value=True)
            self._manual[axis] = tk.StringVar(value="1")
            ttk.Checkbutton(scale, text="自动量程", variable=self._auto[axis], command=self._invalidate).pack(side="left")
            ttk.Label(scale, text="±").pack(side="left")
            entry = ttk.Entry(scale, textvariable=self._manual[axis], width=6)
            entry.pack(side="left", padx=2)
            entry.bind("<Return>", self._invalidate)
            entry.bind("<FocusOut>", self._invalidate)
            canvas = tk.Canvas(card, width=260, height=190, highlightthickness=0, bg=PALETTE["surface"])
            canvas.pack(fill="both", expand=True)
            canvas.bind("<Configure>", self._invalidate)
            canvas.bind("<MouseWheel>", lambda event: scroll.yview_scroll(int(-event.delta/120), "units"))
            self._canvases[axis] = canvas
        row.columnconfigure(3, weight=1, uniform="debug_axis")
        row.rowconfigure(0, weight=1)
        gcard = ttk.Frame(row, padding=(3, 3))
        gcard.grid(row=0, column=3, sticky="nsew")
        controls = ttk.Frame(gcard)
        controls.pack(fill="x")
        ttk.Label(controls, text="有符号g矢量").pack(side="left")
        scale = ttk.Frame(gcard)
        scale.pack(fill="x", pady=1)
        self._auto["g"] = tk.BooleanVar(value=True)
        self._manual["g"] = tk.StringVar(value="1")
        ttk.Checkbutton(scale, text="自动量程", variable=self._auto["g"], command=self._invalidate).pack(side="left")
        ttk.Label(scale, text="±g").pack(side="left")
        entry = ttk.Entry(scale, textvariable=self._manual["g"], width=6)
        entry.pack(side="left", padx=2)
        entry.bind("<Return>", self._invalidate)
        entry.bind("<FocusOut>", self._invalidate)
        gcanvas = tk.Canvas(gcard, width=260, height=190, highlightthickness=0, bg=PALETTE["surface"])
        gcanvas.pack(fill="both", expand=True)
        gcanvas.bind("<Configure>", self._invalidate)
        gcanvas.bind("<MouseWheel>", lambda event: scroll.yview_scroll(int(-event.delta/120), "units"))
        self._canvases["g"] = gcanvas
        # Detailed information stays on a separate tab; it cannot push plots off
        # the 768-pixel display.  Both directions can scroll on narrow windows.
        details = tk.Canvas(detail, bg=PALETTE["surface"], highlightthickness=0)
        ybar = ttk.Scrollbar(detail, orient="vertical", command=details.yview)
        xbar2 = ttk.Scrollbar(detail, orient="horizontal", command=details.xview)
        details.configure(yscrollcommand=ybar.set, xscrollcommand=xbar2.set, scrollregion=(0, 0, 1250, 1100))
        ybar.pack(side="right", fill="y")
        xbar2.pack(side="bottom", fill="x")
        details.pack(fill="both", expand=True)
        self._canvases["states"] = details
        details.bind("<Configure>", self._invalidate)
        details.bind("<MouseWheel>", lambda event: details.yview_scroll(int(-event.delta/120), "units"))

    def show(self):
        if self.window is None or not self.window.winfo_exists():
            self._build()
        self._shown = True
        if self.parent is None:
            self.window.deiconify()
            self.window.lift()
        else:
            self.window.pack(fill="both", expand=True)
        self._last_key = None
        self.refresh(self.displayed_time_provider())

    def hide(self):
        self._shown = False
        if self.window is not None and self.window.winfo_exists():
            if self.parent is None:
                self.window.withdraw()
            else:
                self.window.pack_forget()

    def close(self):
        self._shown = False
        if self.window is not None:
            try:
                self.window.destroy()
            except Exception:
                pass
        self.window = None
        self._canvases.clear()
        self._last_key = None

    def _invalidate(self, _event=None):
        self._last_key = None

    def _settings(self):
        ranges = {}
        errors = []
        for key, automatic in self._auto.items():
            if automatic.get():
                ranges[key] = None
                continue
            try:
                value = float(self._manual[key].get())
                if not math.isfinite(value) or value <= 0:
                    raise ValueError
                ranges[key] = value
            except ValueError:
                ranges[key] = None
                errors.append((AXIS_LABELS.get(key, "g矢量"))+"量程无效，暂用自动")
        try:
            window_s = float(self._time_window.get())
            if not math.isfinite(window_s) or window_s <= 0:
                raise ValueError
        except ValueError:
            window_s = 3.
            errors.append("时间窗无效，暂用3秒")
        return ({key: value.get() for key, value in self._modes.items()},
                {key: value.get() for key, value in self._visible.items()}, ranges, window_s, errors)

    def refresh(self, displayed_t, *, force=False):
        if not _finite(displayed_t):
            return
        # The caller may request an unsnapped replay cursor; the provider is the
        # authoritative restored state time.  Taking min forbids future samples.
        provider_t = self.displayed_time_provider()
        self._displayed_t = min(float(displayed_t), float(provider_t)) if _finite(provider_t) else float(displayed_t)
        if (not self._shown or self.window is None or not self.window.winfo_exists()
                or not self.window.winfo_ismapped()):
            return
        key = (self.buffer.generation, self._displayed_t)
        if key == self._last_key:
            return
        now = time.perf_counter()
        if not force and self._last_key is not None and now-getattr(self, "_last_paint_wall", 0.) < 1./12.:
            return
        modes, visible, ranges, window_s, errors = self._settings()
        records, count = self.buffer._window_view(self._displayed_t, window_s)
        if not (self.stacked or self.compact) or self.notebook.index(self.notebook.select()) == 0:
            for axis in AXIS_LABELS:
                canvas = self._canvases[axis]
                canvas.delete("all")
                draw_axis(_CanvasPainter(canvas), max(1, canvas.winfo_width()),
                          max(1, canvas.winfo_height()),
                          records, axis, modes[axis], visible, ranges[axis], window_s, dense=self.stacked)
            if "g" in self._canvases:
                canvas = self._canvases["g"]
                canvas.delete("all")
                draw_g_vector(_CanvasPainter(canvas), max(1, canvas.winfo_width()),
                              max(1, canvas.winfo_height()),
                              records[-1] if records else None, visible, ranges["g"])
        if not (self.stacked or self.compact) or self.notebook.index(self.notebook.select()) == 1:
            canvas = self._canvases["states"]
            canvas.delete("all")
            draw_states(_CanvasPainter(canvas), max(700 if self.compact else 400, canvas.winfo_width()), max(1100, canvas.winfo_height()),
                        records[-1] if records else None)
        last = _fmt(records[-1]["t_s"], 6) if records else "无"
        text = (f"t={self._displayed_t:.6f}s · {count}条 · 截断{self.buffer.dropped_samples}" if self.stacked
                else f"游标 {self._displayed_t:.6f}s · 样本t {last} · {count}条 · 截断{self.buffer.dropped_samples}")
        if errors:
            text += " · 检查量程/窗口" if self.stacked else " · "+"；".join(errors)
        self._status.set(text)
        self._last_key = key
        self._last_paint_wall = time.perf_counter()

    def _cutoff(self):
        now = self.displayed_time_provider()
        return min(self._displayed_t, float(now)) if _finite(now) else self._displayed_t

    def _path(self, suffix):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return OUTPUTS / f"flight_debug_{stamp}{suffix}"

    def _notify(self, path):
        from tkinter import messagebox
        messagebox.showinfo("导出完成", str(path), parent=self.window)

    def _export_csv(self):
        from tkinter import messagebox
        try:
            self._notify(self.buffer.export_csv(self._path(".csv"), until=self._cutoff()))
        except (OSError, ValueError) as exc:
            messagebox.showerror("导出失败", str(exc), parent=self.window)

