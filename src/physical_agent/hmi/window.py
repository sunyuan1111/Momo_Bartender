"""Single-window Quick Move GUI for the STS3215 arm controller."""

from __future__ import annotations

import time
from datetime import datetime
from pathlib import Path

from PyQt5.QtCore import QObject, QThread, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import (
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from physical_agent.controller import Sts3215ArmController

from .quick_move_page import QuickMovePage
from .theme import get_stylesheet
from .widgets import GlobalStatusBar


def _format_joint_summary(values: dict[str, float] | None) -> str:
    if not values:
        return "--"
    return ", ".join(f"{name}={value:.2f}deg" for name, value in values.items())


class ControllerWorker(QObject):
    busy_changed = pyqtSignal(bool)
    state_ready = pyqtSignal(object)
    connection_changed = pyqtSignal(bool, str)
    error = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._controller: Sts3215ArmController | None = None
        self._config_path = ""

    def _set_busy(self, busy: bool):
        self.busy_changed.emit(bool(busy))

    def _emit_state(self):
        controller = self._controller
        if controller is None or not controller.is_connected:
            self.state_ready.emit(None)
            return

        state = controller.read_state()
        payload = {
            "config_path": self._config_path,
            "port": controller.config.port,
            "joint_names": [joint.name for joint in controller.config.joints],
            "cartesian_enabled": controller.config.has_urdf_kinematics,
            "default_speed_deg_s": controller.config.default_speed_deg_s,
            "state": state,
            "timestamp": time.time(),
        }
        self.state_ready.emit(payload)

    def _require_controller(self) -> Sts3215ArmController:
        controller = self._controller
        if controller is None or not controller.is_connected:
            raise RuntimeError("Robot is not connected.")
        return controller

    @pyqtSlot(str)
    def connect_robot(self, config_path: str):
        self._set_busy(True)
        try:
            self._disconnect_robot_impl(emit_busy=False)
            controller = Sts3215ArmController.from_json(config_path)
            controller.connect()
            self._controller = controller
            self._config_path = config_path
            self.connection_changed.emit(True, "")
            self._emit_state()
        except Exception as exc:
            self._controller = None
            self._config_path = config_path
            self.connection_changed.emit(False, str(exc))
            self.error.emit(str(exc))
        finally:
            self._set_busy(False)

    @pyqtSlot()
    def disconnect_robot(self):
        self._disconnect_robot_impl(emit_busy=True)

    def _disconnect_robot_impl(self, *, emit_busy: bool):
        if emit_busy:
            self._set_busy(True)
        try:
            if self._controller is not None:
                try:
                    self._controller.disconnect()
                finally:
                    self._controller = None
            self.connection_changed.emit(False, "")
            self.state_ready.emit(None)
        except Exception as exc:
            self.error.emit(str(exc))
        finally:
            if emit_busy:
                self._set_busy(False)

    @pyqtSlot()
    def refresh_state(self):
        try:
            self._emit_state()
        except Exception as exc:
            self.error.emit(str(exc))

    @pyqtSlot(float)
    def move_home(self, speed_deg_s: float):
        self._set_busy(True)
        try:
            controller = self._require_controller()
            controller.home(speed_deg_s=speed_deg_s)
            self._emit_state()
        except Exception as exc:
            self.error.emit(str(exc))
        finally:
            self._set_busy(False)

    @pyqtSlot(str, float, float)
    def step_joint(self, joint_name: str, delta_deg: float, speed_deg_s: float):
        self._set_busy(True)
        try:
            controller = self._require_controller()
            controller.nudge_joint(joint_name, delta_deg, speed_deg_s=speed_deg_s)
            self._emit_state()
        except Exception as exc:
            self.error.emit(str(exc))
        finally:
            self._set_busy(False)

    @pyqtSlot(float, float, float, float)
    def jog_cartesian(self, dx_m: float, dy_m: float, dz_m: float, speed_deg_s: float):
        self._set_busy(True)
        try:
            controller = self._require_controller()
            controller.nudge_cartesian(dx_m, dy_m, dz_m, speed_deg_s=speed_deg_s)
            self._emit_state()
        except Exception as exc:
            self.error.emit(str(exc))
        finally:
            self._set_busy(False)


class QuickMoveWindow(QMainWindow):
    request_connect = pyqtSignal(str)
    request_disconnect = pyqtSignal()
    request_refresh = pyqtSignal()
    request_home = pyqtSignal(float)
    request_joint_step = pyqtSignal(str, float, float)
    request_cartesian_jog = pyqtSignal(float, float, float, float)

    def __init__(self, config_path: Path, refresh_interval_ms: int = 500):
        super().__init__()
        self.setWindowTitle("Physical Agent Quick Move")
        self.setMinimumSize(1280, 820)
        self.setStyleSheet(get_stylesheet())

        self._config_path = str(config_path)
        self._connected = False
        self._busy = False
        self._cartesian_enabled = False
        self._base_speed_deg_s = 10.0
        self._last_message = "Ready"
        self._jog_hold_key: str | None = None

        self._worker_thread = QThread(self)
        self._worker = ControllerWorker()
        self._worker.moveToThread(self._worker_thread)
        self._worker_thread.start()

        self._build_ui()
        self._bind_worker_signals()

        self._refresh_timer = QTimer(self)
        self._refresh_timer.setInterval(max(100, int(refresh_interval_ms)))
        self._refresh_timer.timeout.connect(self._request_refresh_if_idle)
        self._refresh_timer.start()

        self._jog_hold_timer = QTimer(self)
        self._jog_hold_timer.setInterval(180)
        self._jog_hold_timer.timeout.connect(self._on_jog_hold_tick)

        self._update_header_state()
        self._update_global_status_bar()
        self.page.set_runtime_state(
            runtime="Disconnected",
            port="--",
            config_path=self._config_path,
            cartesian="Disabled",
            last_update="--",
            present_summary="--",
            target_summary="--",
        )

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 0)
        root.setSpacing(8)

        header = QWidget()
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(4, 4, 4, 4)
        header_layout.setSpacing(8)

        header_layout.addWidget(QLabel("Config"))
        self.config_edit = QLineEdit(self._config_path)
        header_layout.addWidget(self.config_edit, stretch=1)

        self.browse_btn = QPushButton("Browse")
        self.browse_btn.clicked.connect(self._on_browse_clicked)
        header_layout.addWidget(self.browse_btn)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("primaryBtn")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        header_layout.addWidget(self.connect_btn)

        self.header_status = QLabel("Disconnected")
        self.header_status.setObjectName("valueLabel")
        header_layout.addWidget(self.header_status)

        root.addWidget(header)

        self.page = QuickMovePage()
        root.addWidget(self.page, stretch=1)

        self.global_status = GlobalStatusBar()
        root.addWidget(self.global_status)

        self.page.refresh_clicked.connect(self._request_refresh_if_idle)
        self.page.home_clicked.connect(self._request_home)
        self.page.joint_step_requested.connect(self._request_joint_step)
        self.page.cartesian_jog_pressed.connect(self._on_cartesian_jog_pressed)
        self.page.cartesian_jog_released.connect(self._on_cartesian_jog_released)
        self.page.speed_changed.connect(lambda _value: self._update_global_status_bar())

    def _bind_worker_signals(self):
        self.request_connect.connect(self._worker.connect_robot)
        self.request_disconnect.connect(self._worker.disconnect_robot)
        self.request_refresh.connect(self._worker.refresh_state)
        self.request_home.connect(self._worker.move_home)
        self.request_joint_step.connect(self._worker.step_joint)
        self.request_cartesian_jog.connect(self._worker.jog_cartesian)

        self._worker.busy_changed.connect(self._on_worker_busy_changed)
        self._worker.connection_changed.connect(self._on_connection_changed)
        self._worker.state_ready.connect(self._on_state_ready)
        self._worker.error.connect(self._on_worker_error)

    def _resolved_speed_deg_s(self) -> float:
        speed_scale = max(0.01, float(self.page.speed_slider.value()) / 100.0)
        return max(0.5, self._base_speed_deg_s * speed_scale)

    def _request_refresh_if_idle(self):
        if self._busy:
            return
        self.request_refresh.emit()

    def _request_home(self):
        if not self._connected or self._busy:
            return
        self.request_home.emit(self._resolved_speed_deg_s())

    def _request_joint_step(self, joint_name: str, direction: float):
        if not self._connected or self._busy:
            return
        step_deg = max(0.1, float(self.page.step_angle_spin.value()))
        self.request_joint_step.emit(str(joint_name), float(direction) * step_deg, self._resolved_speed_deg_s())

    def _cartesian_delta_from_key(self, key: str) -> tuple[float, float, float] | None:
        step_mm = max(0.1, float(self.page.step_dist_spin.value()))
        step_m = step_mm / 1000.0
        mapping = {
            "+X": (step_m, 0.0, 0.0),
            "-X": (-step_m, 0.0, 0.0),
            "+Y": (0.0, step_m, 0.0),
            "-Y": (0.0, -step_m, 0.0),
            "+Z": (0.0, 0.0, step_m),
            "-Z": (0.0, 0.0, -step_m),
        }
        return mapping.get(str(key).strip().upper())

    def _quick_jog_mode_is_continuous(self) -> bool:
        return self.page.step_mode_combo.currentIndex() == 1

    def _request_cartesian_jog(self, key: str):
        if not self._connected or self._busy or not self._cartesian_enabled:
            return
        delta = self._cartesian_delta_from_key(key)
        if delta is None:
            return
        self.request_cartesian_jog.emit(delta[0], delta[1], delta[2], self._resolved_speed_deg_s())

    def _on_cartesian_jog_pressed(self, key: str):
        if not self._quick_jog_mode_is_continuous():
            self._request_cartesian_jog(key)
            return
        self._jog_hold_key = str(key)
        self._request_cartesian_jog(self._jog_hold_key)
        self._jog_hold_timer.start()

    def _on_cartesian_jog_released(self):
        self._jog_hold_key = None
        if self._jog_hold_timer.isActive():
            self._jog_hold_timer.stop()

    def _on_jog_hold_tick(self):
        if not self._jog_hold_key:
            self._jog_hold_timer.stop()
            return
        self._request_cartesian_jog(self._jog_hold_key)

    def _on_browse_clicked(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Select config",
            str(Path(self.config_edit.text()).expanduser().resolve().parent),
            "JSON (*.json)",
        )
        if path:
            self.config_edit.setText(path)

    def _on_connect_clicked(self):
        if self._busy:
            return
        if self._connected:
            self._on_cartesian_jog_released()
            self.request_disconnect.emit()
            return
        self._config_path = str(Path(self.config_edit.text()).expanduser())
        self.request_connect.emit(self._config_path)

    def _on_worker_busy_changed(self, busy: bool):
        self._busy = bool(busy)
        self._update_header_state()

    def _on_connection_changed(self, connected: bool, message: str):
        self._connected = bool(connected)
        if connected:
            self._last_message = "Connected"
            self.page.set_status_light("normal")
        else:
            self._last_message = "Disconnected" if not message else message
            self.page.set_status_light("warning" if message else "normal")
            self._cartesian_enabled = False
            self._base_speed_deg_s = 10.0
            self.page.set_joint_names(())
            self.page.set_joint_values({})
            self.page.set_pose_values_mm(None)
            self.page.set_motion_enabled(False)
            self.page.set_cartesian_enabled(False)
            self.page.set_runtime_state(
                runtime="Disconnected",
                port="--",
                config_path=self.config_edit.text().strip() or "--",
                cartesian="Disabled",
                last_update="--",
                present_summary="--",
                target_summary="--",
            )
        self._update_header_state()
        self._update_global_status_bar()

    def _on_state_ready(self, payload: object):
        if payload is None:
            return
        data = dict(payload)
        state = dict(data["state"])
        present = dict(state.get("present_positions_deg", {}))
        targets = dict(state.get("target_positions_deg", {}))
        joint_names = list(data.get("joint_names", []))
        cartesian_enabled = bool(data.get("cartesian_enabled", False))
        self._cartesian_enabled = cartesian_enabled
        default_speed = data.get("default_speed_deg_s")
        self._base_speed_deg_s = 10.0 if default_speed is None else max(0.5, float(default_speed))
        self.page.set_joint_names(joint_names)
        self.page.set_joint_values(present)
        self.page.set_motion_enabled(self._connected)
        self.page.set_cartesian_enabled(self._connected and cartesian_enabled)

        pose = state.get("cartesian_position_m")
        if isinstance(pose, dict):
            self.page.set_pose_values_mm(
                {
                    "X": float(pose.get("x", 0.0)) * 1000.0,
                    "Y": float(pose.get("y", 0.0)) * 1000.0,
                    "Z": float(pose.get("z", 0.0)) * 1000.0,
                }
            )
        else:
            self.page.set_pose_values_mm(None)

        timestamp = datetime.fromtimestamp(float(data["timestamp"])).strftime("%H:%M:%S")
        self.page.set_runtime_state(
            runtime="Connected",
            port=str(data.get("port", "--")),
            config_path=str(data.get("config_path", "--")),
            cartesian="Enabled" if cartesian_enabled else "Disabled (missing urdf_path)",
            last_update=timestamp,
            present_summary=_format_joint_summary(present),
            target_summary=_format_joint_summary(targets),
        )
        self._last_message = f"Updated {timestamp}"
        self.page.set_status_light("normal")
        self._update_global_status_bar()

    def _on_worker_error(self, message: str):
        self._last_message = str(message).strip() or "Command failed"
        self.page.set_status_light("fault")
        self._update_global_status_bar()

    def _update_header_state(self):
        self.connect_btn.setEnabled(not self._busy)
        self.browse_btn.setEnabled(not self._busy and not self._connected)
        self.config_edit.setReadOnly(self._busy or self._connected)

        if self._busy:
            self.connect_btn.setText("Working...")
        elif self._connected:
            self.connect_btn.setText("Disconnect")
        else:
            self.connect_btn.setText("Connect")

        if self._busy:
            self.header_status.setText("Working")
        elif self._connected:
            self.header_status.setText("Connected")
        else:
            self.header_status.setText("Disconnected")

    def _update_global_status_bar(self):
        self.global_status.set_connection(f"Connection: {'Connected' if self._connected else 'Disconnected'}")
        self.global_status.set_config(f"Config: {Path(self.config_edit.text()).name or '--'}")
        self.global_status.set_cartesian(
            "Cartesian: Enabled" if self._connected and self._cartesian_enabled else "Cartesian: Disabled"
        )
        self.global_status.set_speed(f"Speed: {self.page.speed_slider.value()}%")
        self.global_status.set_message(f"Message: {self._last_message}")

    def closeEvent(self, event):
        self._on_cartesian_jog_released()
        if self._connected:
            self.request_disconnect.emit()
        self._worker_thread.quit()
        self._worker_thread.wait(3000)
        super().closeEvent(event)
