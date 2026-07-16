"""Smart Place Point lightweight BLE IMU desktop application."""

from __future__ import annotations

from collections import deque
import csv
from datetime import datetime
import math
from pathlib import Path
import queue
import random
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import time
from typing import Any

from attitude_solver import AttitudeEstimator, BoardState
from ble_transport import BleTransport
from imu_calibration import CalibrationResult, ImuCalibrator
from imu_protocol import FrameParser, ImuSample, encode_test_frame


APP_DIR = Path(__file__).resolve().parent
PLOT_POINTS = 360
PLOT_REFRESH_MS = 50
EVENT_POLL_MS = 20
COLORS = ("#ff5d73", "#4cc9f0", "#80ed99")


class StripChart(ttk.Frame):
    def __init__(self, master: tk.Misc, title: str, unit: str) -> None:
        super().__init__(master, padding=(8, 6))
        self.title = title
        self.unit = unit
        self.values = tuple(deque(maxlen=PLOT_POINTS) for _ in range(3))
        self.canvas = tk.Canvas(
            self,
            height=225,
            background="#101820",
            highlightthickness=1,
            highlightbackground="#34424e",
        )
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.redraw())

    def append(self, xyz: tuple[float, float, float]) -> None:
        for series, value in zip(self.values, xyz):
            series.append(value)

    def clear(self) -> None:
        for series in self.values:
            series.clear()
        self.redraw()

    def redraw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 200)
        height = max(canvas.winfo_height(), 120)
        left, top, right, bottom = 56, 28, 14, 28
        plot_w = width - left - right
        plot_h = height - top - bottom
        all_values = [value for series in self.values for value in series]
        peak = max((abs(value) for value in all_values), default=1.0)
        # Stable, symmetric autoscaling with a little visual headroom.
        scale = self._nice_limit(max(peak * 1.15, 0.05))

        canvas.create_text(
            12, 10, anchor="nw", text=f"{self.title} ({self.unit})",
            fill="#f2f5f7", font=("Segoe UI", 11, "bold")
        )
        for index, (label, color) in enumerate(zip("XYZ", COLORS)):
            x = width - 142 + index * 44
            canvas.create_line(x, 17, x + 13, 17, fill=color, width=3)
            canvas.create_text(x + 18, 17, text=label, fill="#dce5ea", anchor="w")

        for grid_index in range(5):
            y = top + plot_h * grid_index / 4
            value = scale * (1 - grid_index / 2)
            canvas.create_line(left, y, width - right, y, fill="#273640")
            canvas.create_text(left - 7, y, text=f"{value:.2g}", fill="#91a3ad", anchor="e")

        point_count = max((len(series) for series in self.values), default=0)
        if point_count < 2:
            canvas.create_text(
                left + plot_w / 2, top + plot_h / 2,
                text="等待 IMU 数据…", fill="#71838d"
            )
            return

        denominator = max(PLOT_POINTS - 1, point_count - 1)
        for series, color in zip(self.values, COLORS):
            coordinates: list[float] = []
            x_offset = PLOT_POINTS - len(series)
            for index, value in enumerate(series):
                x = left + (x_offset + index) * plot_w / denominator
                y = top + (scale - value) * plot_h / (2 * scale)
                coordinates.extend((x, y))
            if len(coordinates) >= 4:
                canvas.create_line(*coordinates, fill=color, width=2, smooth=False)

    @staticmethod
    def _nice_limit(value: float) -> float:
        exponent = math.floor(math.log10(value))
        fraction = value / (10**exponent)
        nice_fraction = 1 if fraction <= 1 else 2 if fraction <= 2 else 5 if fraction <= 5 else 10
        return nice_fraction * (10**exponent)


class OrientationCube(ttk.Frame):
    """Render the solved board attitude as a cube in fixed world axes."""

    _VERTICES = (
        (-1.0, -1.0, -1.0), (1.0, -1.0, -1.0),
        (1.0, 1.0, -1.0), (-1.0, 1.0, -1.0),
        (-1.0, -1.0, 1.0), (1.0, -1.0, 1.0),
        (1.0, 1.0, 1.0), (-1.0, 1.0, 1.0),
    )
    _FACES = (
        ((0, 1, 2, 3), "#30475e"),
        ((4, 5, 6, 7), "#4f86c6"),
        ((0, 1, 5, 4), "#315f72"),
        ((1, 2, 6, 5), "#3f7d65"),
        ((2, 3, 7, 6), "#856a43"),
        ((3, 0, 4, 7), "#6d597a"),
    )
    _EDGES = (
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    )
    _AXES = (
        ((2.2, 0.0, 0.0), "X", "#ff5d73"),
        ((0.0, 2.2, 0.0), "Y", "#80ed99"),
        ((0.0, 0.0, 2.2), "Z", "#4cc9f0"),
    )

    def __init__(self, master: tk.Misc) -> None:
        super().__init__(master)
        self.state: BoardState | None = None
        self.canvas = tk.Canvas(
            self,
            width=430,
            height=410,
            background="#101820",
            highlightthickness=1,
            highlightbackground="#34424e",
        )
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.redraw())

    def set_state(self, state: BoardState | None) -> None:
        self.state = state

    def redraw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 300)
        height = max(canvas.winfo_height(), 280)
        center_x, center_y = width * 0.5, height * 0.52
        scale = min(width, height) * 0.21
        state = self.state
        roll = state.roll_deg if state is not None else 0.0
        pitch = state.pitch_deg if state is not None else 0.0
        yaw = state.yaw_deg if state is not None else 0.0

        canvas.create_text(
            14, 12, anchor="nw", text="三维电路板姿态",
            fill="#f2f5f7", font=("Segoe UI", 12, "bold")
        )
        canvas.create_text(
            width - 14, 14, anchor="ne",
            text=f"Roll {roll:+.1f}°   Pitch {pitch:+.1f}°   Yaw {yaw:+.1f}°",
            fill="#aab9c2", font=("Consolas", 10)
        )

        origin = self._screen_point((0.0, 0.0, 0.0), center_x, center_y, scale)
        for endpoint, label, color in self._AXES:
            projected = self._screen_point(endpoint, center_x, center_y, scale)
            canvas.create_line(
                origin[0], origin[1], projected[0], projected[1],
                fill=color, width=2, arrow="last", arrowshape=(10, 12, 4)
            )
            canvas.create_text(
                projected[0], projected[1], text=label, fill=color,
                font=("Segoe UI", 11, "bold"), anchor="sw"
            )

        rotated = [self._rotate(vertex, roll, pitch, yaw) for vertex in self._VERTICES]
        projected = [self._screen_point(vertex, center_x, center_y, scale) for vertex in rotated]
        faces = []
        for indices, color in self._FACES:
            depth = sum(self._project(rotated[index])[2] for index in indices) / len(indices)
            faces.append((depth, indices, color))
        for _depth, indices, color in sorted(faces, reverse=True):
            coordinates = [coordinate for index in indices for coordinate in projected[index][:2]]
            canvas.create_polygon(coordinates, fill=color, outline="#17232c", width=1)
        for start, end in self._EDGES:
            canvas.create_line(
                projected[start][0], projected[start][1],
                projected[end][0], projected[end][1],
                fill="#dce5ea", width=2,
            )

        canvas.create_text(
            center_x, height - 18, anchor="s",
            text="世界坐标系：X 红 / Y 绿 / Z 蓝",
            fill="#82949e", font=("Segoe UI", 9)
        )
        if state is None:
            canvas.create_text(
                center_x, center_y, text="等待 IMU 数据…", fill="#ffffff",
                font=("Segoe UI", 13, "bold")
            )

    @staticmethod
    def _rotate(
        point: tuple[float, float, float], roll_deg: float, pitch_deg: float, yaw_deg: float
    ) -> tuple[float, float, float]:
        x, y, z = point
        roll, pitch, yaw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        y, z = y * cr - z * sr, y * sr + z * cr
        x, z = x * cp + z * sp, -x * sp + z * cp
        x, y = x * cy - y * sy, x * sy + y * cy
        return x, y, z

    @staticmethod
    def _project(point: tuple[float, float, float]) -> tuple[float, float, float]:
        # Orthographic camera at approximately (4, -6, 4), looking at the origin.
        x, y, z = point
        right = (0.8320503, 0.5547002, 0.0)
        up = (-0.2844273, 0.4266410, 0.8588975)
        forward = (-0.4850713, 0.7276069, -0.4850713)
        return (
            x * right[0] + y * right[1] + z * right[2],
            x * up[0] + y * up[1] + z * up[2],
            x * forward[0] + y * forward[1] + z * forward[2],
        )

    @classmethod
    def _screen_point(
        cls,
        point: tuple[float, float, float],
        center_x: float,
        center_y: float,
        scale: float,
    ) -> tuple[float, float, float]:
        horizontal, vertical, depth = cls._project(point)
        return center_x + horizontal * scale, center_y - vertical * scale, depth


class CsvRecorder:
    HEADER = (
        "timestamp", "elapsed_s", "temperature_c",
        "accel_x_g", "accel_y_g", "accel_z_g",
        "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
        "roll_deg", "pitch_deg", "yaw_relative_deg",
        "accel_magnitude_g", "angular_rate_dps", "motion", "orientation",
        "raw_temp", "raw_ax", "raw_ay", "raw_az",
        "raw_gx", "raw_gy", "raw_gz",
    )

    def __init__(self) -> None:
        self._file: Any = None
        self._writer: Any = None
        self._start_time: float | None = None
        self.path: Path | None = None

    @property
    def active(self) -> bool:
        return self._file is not None

    def start(self, path: Path, start_time: float | None) -> None:
        self.stop()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._file = path.open("w", newline="", encoding="utf-8-sig")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADER)
        self._start_time = start_time
        self.path = path

    def write(self, sample: ImuSample, state: BoardState) -> None:
        if self._writer is None:
            return
        if self._start_time is None:
            self._start_time = sample.received_at
        self._writer.writerow(
            (
                sample.wall_time.isoformat(timespec="milliseconds"),
                f"{sample.received_at - self._start_time:.6f}",
                f"{sample.temperature_c:.4f}",
                *(f"{value:.7f}" for value in sample.accel_g),
                *(f"{value:.5f}" for value in sample.gyro_dps),
                f"{state.roll_deg:.5f}",
                f"{state.pitch_deg:.5f}",
                f"{state.yaw_deg:.5f}",
                f"{state.accel_magnitude_g:.7f}",
                f"{state.angular_rate_dps:.5f}",
                state.motion,
                state.orientation,
                *sample.raw,
            )
        )

    def stop(self) -> None:
        if self._file is not None:
            self._file.flush()
            self._file.close()
        self._file = None
        self._writer = None
        self._start_time = None


class SmartPlacePointApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Smart Place Point — IMU Monitor")
        self.root.geometry("1080x760")
        self.root.minsize(820, 620)

        self.events: queue.SimpleQueue[tuple[str, Any]] = queue.SimpleQueue()
        self.parser = FrameParser()
        self.attitude = AttitudeEstimator()
        self.calibrator = ImuCalibrator()
        self.transport = BleTransport(lambda kind, payload: self.events.put((kind, payload)))
        self.recorder = CsvRecorder()
        self.devices_by_label: dict[str, str] = {}
        self.connected = False
        self.demo_running = False
        self.demo_step = 0
        self.latest_sample: ImuSample | None = None
        self.board_state: BoardState | None = None
        self.state_window: tk.Toplevel | None = None
        self.orientation_cube: OrientationCube | None = None
        self.state_vars: dict[str, tk.StringVar] = {}
        self.recent_frame_times: deque[float] = deque()

        self.status_var = tk.StringVar(value="就绪")
        self.device_var = tk.StringVar()
        self.stats_var = tk.StringVar(value="帧: 0   速率: 0.0 Hz   校验错误: 0   丢弃字节: 0")
        self.temp_var = tk.StringVar(value="-- °C")
        self.accel_vars = [tk.StringVar(value="--") for _ in range(3)]
        self.gyro_vars = [tk.StringVar(value="--") for _ in range(3)]

        self._build_ui()
        self._set_connection_controls(False)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(EVENT_POLL_MS, self._poll_events)
        self.root.after(PLOT_REFRESH_MS, self._refresh_plots)

    def _build_ui(self) -> None:
        style = ttk.Style()
        if "vista" in style.theme_names():
            style.theme_use("vista")

        toolbar = ttk.Frame(self.root, padding=10)
        toolbar.pack(fill="x")
        self.scan_button = ttk.Button(toolbar, text="扫描设备", command=self._scan)
        self.scan_button.pack(side="left")
        self.device_box = ttk.Combobox(
            toolbar, textvariable=self.device_var, state="readonly", width=32
        )
        self.device_box.pack(side="left", padx=(8, 8), fill="x", expand=True)
        self.connect_button = ttk.Button(toolbar, text="连接", command=self._connect)
        self.connect_button.pack(side="left")
        self.disconnect_button = ttk.Button(toolbar, text="断开", command=self.transport.disconnect)
        self.disconnect_button.pack(side="left", padx=(6, 0))
        self.demo_button = ttk.Button(toolbar, text="演示模式", command=self._toggle_demo)
        self.demo_button.pack(side="left", padx=(14, 0))
        self.calibration_button = ttk.Button(toolbar, text="校准", command=self._toggle_calibration)
        self.calibration_button.pack(side="left", padx=(6, 0))
        self.state_button = ttk.Button(toolbar, text="电路板状态", command=self._show_board_state)
        self.state_button.pack(side="left", padx=(6, 0))
        self.record_button = ttk.Button(toolbar, text="开始记录", command=self._toggle_recording)
        self.record_button.pack(side="left", padx=(6, 0))

        summary = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        summary.pack(fill="x")
        ttk.Label(summary, text="温度", foreground="#5a6770").grid(row=0, column=0, sticky="w")
        ttk.Label(summary, textvariable=self.temp_var, font=("Segoe UI", 18, "bold")).grid(
            row=1, column=0, rowspan=2, sticky="w", padx=(0, 30)
        )
        self._build_value_group(summary, 1, "加速度 (g)", self.accel_vars)
        self._build_value_group(summary, 5, "角速度 (°/s)", self.gyro_vars)
        summary.columnconfigure(4, weight=1)
        summary.columnconfigure(8, weight=1)

        plot_area = ttk.Frame(self.root, padding=(10, 0, 10, 0))
        plot_area.pack(fill="both", expand=True)
        self.accel_chart = StripChart(plot_area, "加速度", "g")
        self.accel_chart.pack(fill="both", expand=True)
        self.gyro_chart = StripChart(plot_area, "角速度", "°/s")
        self.gyro_chart.pack(fill="both", expand=True, pady=(6, 0))

        footer = ttk.Frame(self.root, padding=(10, 7))
        footer.pack(fill="x")
        ttk.Label(footer, textvariable=self.status_var).pack(side="left")
        ttk.Label(footer, textvariable=self.stats_var).pack(side="right")

    @staticmethod
    def _build_value_group(
        parent: ttk.Frame, column: int, title: str, variables: list[tk.StringVar]
    ) -> None:
        ttk.Label(parent, text=title, foreground="#5a6770").grid(
            row=0, column=column, columnspan=3, sticky="w"
        )
        for offset, (axis, variable, color) in enumerate(zip("XYZ", variables, COLORS)):
            ttk.Label(parent, text=axis, foreground=color).grid(row=1, column=column + offset, sticky="w")
            ttk.Label(parent, textvariable=variable, font=("Consolas", 12, "bold"), width=11).grid(
                row=2, column=column + offset, sticky="w", padx=(0, 4)
            )

    def _set_connection_controls(self, connected: bool) -> None:
        self.connected = connected
        self.scan_button.configure(state="disabled" if connected else "normal")
        self.connect_button.configure(state="disabled" if connected else "normal")
        self.disconnect_button.configure(state="normal" if connected else "disabled")

    def _scan(self) -> None:
        if not self.transport.dependency_available:
            messagebox.showerror("缺少依赖", "尚未安装 Bleak。请先在 Software 目录运行 setup.ps1。")
            return
        self.scan_button.configure(state="disabled")
        self.status_var.set("正在扫描 BLE 设备…")
        self.transport.scan()

    def _connect(self) -> None:
        label = self.device_var.get()
        address = self.devices_by_label.get(label)
        if not address:
            messagebox.showinfo("选择设备", "请先扫描并选择 BLE 设备。")
            return
        self.connect_button.configure(state="disabled")
        if self.demo_running:
            self._toggle_demo()
        self.parser.reset()
        self.calibrator.reset()
        self._reset_attitude()
        self.transport.connect(address)

    def _toggle_demo(self) -> None:
        self.demo_running = not self.demo_running
        self.demo_button.configure(text="停止演示" if self.demo_running else "演示模式")
        if self.demo_running:
            if self.connected:
                self.transport.disconnect()
            self.calibrator.reset()
            self.calibration_button.configure(text="校准")
            self._reset_attitude()
            self.status_var.set("演示模式：正在生成模拟 IMU 数据")
            self.demo_step = 0
            self.root.after(10, self._generate_demo_sample)
        else:
            if self.calibrator.collecting:
                self.calibrator.cancel()
                self.calibration_button.configure(text="校准")
            self.status_var.set("演示模式已停止")

    def _generate_demo_sample(self) -> None:
        if not self.demo_running:
            return
        t = self.demo_step / 50.0
        values = (
            int((28.0 - 25.0) * 132.48),
            int((0.25 * math.sin(t * 2.2) + random.uniform(-0.015, 0.015)) * 2048),
            int((0.18 * math.sin(t * 1.4 + 1.0) + random.uniform(-0.015, 0.015)) * 2048),
            int((1.0 + 0.05 * math.sin(t * 0.8) + random.uniform(-0.01, 0.01)) * 2048),
            int(80 * math.sin(t * 1.7) * 16.4),
            int(55 * math.sin(t * 1.1 + 0.8) * 16.4),
            int(30 * math.sin(t * 0.7 + 1.8) * 16.4),
        )
        self._consume_bytes(encode_test_frame(values))
        self.demo_step += 1
        self.root.after(20, self._generate_demo_sample)

    def _toggle_recording(self) -> None:
        if self.recorder.active:
            path = self.recorder.path
            self.recorder.stop()
            self.record_button.configure(text="开始记录")
            self.status_var.set(f"记录已保存：{path}")
            return
        data_dir = APP_DIR / "data"
        data_dir.mkdir(exist_ok=True)
        default_name = f"imu_{datetime.now():%Y%m%d_%H%M%S}.csv"
        selected = filedialog.asksaveasfilename(
            title="保存 IMU 数据",
            initialdir=data_dir,
            initialfile=default_name,
            defaultextension=".csv",
            filetypes=(("CSV 文件", "*.csv"), ("所有文件", "*.*")),
        )
        if not selected:
            return
        start_time = self.latest_sample.received_at if self.latest_sample else None
        self.recorder.start(Path(selected), start_time)
        self.record_button.configure(text="停止记录")
        self.status_var.set(f"正在记录：{selected}")

    def _toggle_calibration(self) -> None:
        if self.calibrator.collecting:
            self.calibrator.cancel()
            self.calibration_button.configure(text="校准")
            self.status_var.set("校准已取消，原校准参数保持不变")
            self._update_state_window()
            return

        if self.latest_sample is None or time.perf_counter() - self.latest_sample.received_at > 1.0:
            messagebox.showinfo("无法校准", "请先连接电路板并确认正在接收 IMU 数据。")
            return
        confirmed = messagebox.askokcancel(
            "IMU 静止校准",
            "请将电路板水平平放，+Z 板面朝上，并在接下来的 2 秒内保持完全静止。\n\n"
            "点击“确定”开始采集。",
        )
        if not confirmed:
            return
        self.calibrator.start()
        self.calibration_button.configure(text="取消校准")
        self.status_var.set("正在校准：请保持电路板水平静止…")
        self._update_state_window()

    def _finish_calibration(self, result: CalibrationResult) -> None:
        self.calibration_button.configure(text="校准")
        if result.success:
            self.attitude.reset()
            self.status_var.set(result.message)
            messagebox.showinfo(
                "校准成功",
                f"{result.message}\n\n"
                f"加速度零偏 (g)：{self._format_vector(result.accel_bias_g, 5)}\n"
                f"角速度零偏 (°/s)：{self._format_vector(result.gyro_bias_dps, 4)}",
            )
        else:
            self.status_var.set(f"校准失败：{result.message}")
            messagebox.showwarning("校准失败", result.message)

    @staticmethod
    def _format_vector(values: tuple[float, float, float] | None, digits: int) -> str:
        if values is None:
            return "--"
        return "  ".join(f"{axis}={value:+.{digits}f}" for axis, value in zip("XYZ", values))

    def _show_board_state(self) -> None:
        if self.state_window is not None and self.state_window.winfo_exists():
            self.state_window.deiconify()
            self.state_window.lift()
            self._update_state_window()
            return

        window = tk.Toplevel(self.root)
        self.state_window = window
        window.title("当前电路板状态")
        window.geometry("920x590")
        window.minsize(760, 520)
        window.transient(self.root)
        window.protocol("WM_DELETE_WINDOW", self._close_state_window)

        content = ttk.Frame(window, padding=18)
        content.pack(fill="both", expand=True)
        ttk.Label(content, text="六轴 IMU 姿态解算", font=("Segoe UI", 15, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 12)
        )

        self.orientation_cube = OrientationCube(content)
        self.orientation_cube.grid(row=1, column=0, sticky="nsew", padx=(0, 18))

        details = ttk.Frame(content)
        details.grid(row=1, column=1, sticky="nsew")

        rows = (
            ("link", "数据链路"),
            ("temperature", "IMU 温度"),
            ("roll", "横滚角 Roll"),
            ("pitch", "俯仰角 Pitch"),
            ("yaw", "航向相对角 Yaw"),
            ("orientation", "电路板朝向"),
            ("motion", "运动状态"),
            ("accel", "合加速度"),
            ("gyro", "合角速度"),
            ("calibration", "校准状态"),
            ("quality", "数据质量"),
        )
        self.state_vars = {key: tk.StringVar(value="等待 IMU 数据…") for key, _label in rows}
        for row, (key, label) in enumerate(rows):
            ttk.Label(details, text=label, foreground="#5a6770").grid(
                row=row, column=0, sticky="w", padx=(0, 24), pady=5
            )
            ttk.Label(details, textvariable=self.state_vars[key], font=("Segoe UI", 10, "bold")).grid(
                row=row, column=1, sticky="w", pady=5
            )

        ttk.Separator(content).grid(row=2, column=0, columnspan=2, sticky="ew", pady=(12, 8))
        ttk.Label(
            content,
            text="说明：Roll/Pitch 由加速度计与陀螺仪互补滤波获得；\n"
            "Yaw 没有磁力计校正，是上电或重新连接后的相对角度，会随时间漂移。",
            foreground="#6b747a",
            justify="left",
        ).grid(row=3, column=0, columnspan=2, sticky="w")
        content.rowconfigure(1, weight=1)
        content.columnconfigure(0, weight=3)
        content.columnconfigure(1, weight=1)
        self._update_state_window()
        self.orientation_cube.redraw()

    def _close_state_window(self) -> None:
        if self.state_window is not None:
            self.state_window.destroy()
        self.state_window = None
        self.orientation_cube = None
        self.state_vars = {}

    def _reset_attitude(self) -> None:
        self.attitude.reset()
        self.board_state = None
        self._update_state_window()

    def _update_state_window(self) -> None:
        if self.state_window is None or not self.state_window.winfo_exists() or not self.state_vars:
            return
        state = self.board_state
        sample = self.latest_sample
        if state is None or sample is None:
            if self.orientation_cube is not None:
                self.orientation_cube.set_state(None)
            for variable in self.state_vars.values():
                variable.set("等待 IMU 数据…")
            return

        if self.orientation_cube is not None:
            self.orientation_cube.set_state(state)

        age = max(time.perf_counter() - state.received_at, 0.0)
        if age < 0.5:
            link = f"实时接收（{age * 1000:.0f} ms）"
        elif age < 2.0:
            link = f"数据延迟（{age:.1f} s）"
        else:
            link = f"数据已中断（{age:.1f} s）"
        self.state_vars["link"].set(link)
        self.state_vars["temperature"].set(f"{sample.temperature_c:.2f} °C")
        self.state_vars["roll"].set(f"{state.roll_deg:+.2f}°")
        self.state_vars["pitch"].set(f"{state.pitch_deg:+.2f}°")
        self.state_vars["yaw"].set(f"{state.yaw_deg:+.2f}°")
        self.state_vars["orientation"].set(state.orientation)
        self.state_vars["motion"].set(state.motion)
        self.state_vars["accel"].set(f"{state.accel_magnitude_g:.4f} g")
        self.state_vars["gyro"].set(f"{state.angular_rate_dps:.2f} °/s")
        if self.calibrator.collecting:
            calibration = f"采集中 {self.calibrator.progress * 100:.0f}%"
        elif self.calibrator.is_calibrated:
            calibration = "已校准"
        else:
            calibration = "未校准"
        self.state_vars["calibration"].set(calibration)
        self.state_vars["quality"].set(
            f"有效帧 {self.parser.good_frames}，校验错误 {self.parser.checksum_errors}"
        )

    def _poll_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "data":
                    self._consume_bytes(payload)
                elif kind == "devices":
                    self._show_devices(payload)
                elif kind == "connected":
                    self._set_connection_controls(True)
                    self.status_var.set(f"已连接：{payload['name']}")
                elif kind == "disconnected":
                    if self.calibrator.collecting:
                        self.calibrator.cancel()
                        self.calibration_button.configure(text="校准")
                    self._set_connection_controls(False)
                    if not self.demo_running:
                        self.status_var.set("BLE 已断开")
                elif kind == "status":
                    self.status_var.set(str(payload))
                elif kind == "error":
                    self._set_connection_controls(False)
                    self.status_var.set(str(payload))
                    messagebox.showerror("BLE 错误", str(payload))
        except queue.Empty:
            pass
        self.root.after(EVENT_POLL_MS, self._poll_events)

    def _show_devices(self, devices: list[dict[str, Any]]) -> None:
        self.scan_button.configure(state="normal")
        self.devices_by_label.clear()
        labels = []
        for device in devices:
            label = f"{device['name']}  |  {device['address']}  |  {device['rssi']} dBm"
            labels.append(label)
            self.devices_by_label[label] = device["address"]
        self.device_box.configure(values=labels)
        if labels:
            self.device_box.current(0)

    def _consume_bytes(self, data: bytes) -> None:
        for decoded_sample in self.parser.feed(data):
            calibration_result = None
            if self.calibrator.collecting:
                calibration_result = self.calibrator.add_sample(decoded_sample)
            sample = self.calibrator.apply(decoded_sample)
            self.latest_sample = sample
            self.board_state = self.attitude.update(sample)
            self.accel_chart.append(sample.accel_g)
            self.gyro_chart.append(sample.gyro_dps)
            self.temp_var.set(f"{sample.temperature_c:.2f} °C")
            for variable, value in zip(self.accel_vars, sample.accel_g):
                variable.set(f"{value:+.4f}")
            for variable, value in zip(self.gyro_vars, sample.gyro_dps):
                variable.set(f"{value:+.2f}")
            self.recorder.write(sample, self.board_state)
            self.recent_frame_times.append(sample.received_at)
            if self.calibrator.collecting:
                self.status_var.set(
                    f"正在校准：{self.calibrator.progress * 100:.0f}%（请保持水平静止）"
                )
            if calibration_result is not None:
                self._finish_calibration(calibration_result)
        self._update_state_window()
        self._update_statistics()

    def _update_statistics(self) -> None:
        cutoff = time.perf_counter() - 1.0
        while self.recent_frame_times and self.recent_frame_times[0] < cutoff:
            self.recent_frame_times.popleft()
        self.stats_var.set(
            f"帧: {self.parser.good_frames}   速率: {len(self.recent_frame_times):.1f} Hz   "
            f"校验错误: {self.parser.checksum_errors}   丢弃字节: {self.parser.discarded_bytes}"
        )

    def _refresh_plots(self) -> None:
        self.accel_chart.redraw()
        self.gyro_chart.redraw()
        self._update_statistics()
        self._update_state_window()
        if self.orientation_cube is not None:
            self.orientation_cube.redraw()
        self.root.after(PLOT_REFRESH_MS, self._refresh_plots)

    def _on_close(self) -> None:
        self.demo_running = False
        self.recorder.stop()
        self._close_state_window()
        self.transport.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    SmartPlacePointApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
