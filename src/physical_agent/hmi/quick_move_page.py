"""Quick Move page stripped down to the features supported by this repository."""

from __future__ import annotations

from functools import partial
from typing import Mapping, Sequence

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)


class QuickMovePage(QWidget):
    joint_step_requested = pyqtSignal(str, float)
    cartesian_jog_pressed = pyqtSignal(str)
    cartesian_jog_released = pyqtSignal()
    home_clicked = pyqtSignal()
    refresh_clicked = pyqtSignal()
    speed_changed = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self._joint_rows: dict[str, tuple[QLabel, QLabel, QPushButton, QPushButton]] = {}
        self._joint_names: list[str] = []
        self._motion_enabled = False
        self._cartesian_enabled = False

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(10)

        top = QHBoxLayout()
        top.setSpacing(10)
        root.addLayout(top, stretch=1)

        self.left_group = QGroupBox("Cartesian Jog")
        left_layout = QVBoxLayout(self.left_group)
        left_layout.setSpacing(12)
        left_layout.addWidget(self._build_translation_pad())

        step_row = QHBoxLayout()
        self.step_mode_combo = QComboBox()
        self.step_mode_combo.addItems(["Step", "Continuous"])
        self.step_dist_spin = QDoubleSpinBox()
        self.step_dist_spin.setRange(0.1, 200.0)
        self.step_dist_spin.setValue(10.0)
        self.step_dist_spin.setSuffix(" mm")
        self.step_angle_spin = QDoubleSpinBox()
        self.step_angle_spin.setRange(0.1, 180.0)
        self.step_angle_spin.setValue(2.0)
        self.step_angle_spin.setSuffix(" deg")
        step_row.addWidget(self.step_mode_combo)
        step_row.addWidget(self.step_dist_spin)
        step_row.addWidget(self.step_angle_spin)
        left_layout.addLayout(step_row)

        self.cartesian_hint_label = QLabel("Only base-frame X/Y/Z jog is enabled. Orientation jog has been removed.")
        self.cartesian_hint_label.setWordWrap(True)
        self.cartesian_hint_label.setObjectName("captionLabel")
        left_layout.addWidget(self.cartesian_hint_label)
        top.addWidget(self.left_group, stretch=1)

        self.center_group = QGroupBox("Live State")
        center_layout = QVBoxLayout(self.center_group)
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignLeft)

        self.runtime_value = QLabel("--")
        self.runtime_value.setObjectName("valueLabel")
        self.port_value = QLabel("--")
        self.port_value.setObjectName("valueLabel")
        self.config_value = QLabel("--")
        self.config_value.setWordWrap(True)
        self.config_value.setObjectName("valueLabel")
        self.cartesian_value = QLabel("--")
        self.cartesian_value.setObjectName("valueLabel")
        self.last_update_value = QLabel("--")
        self.last_update_value.setObjectName("valueLabel")

        form.addRow("Runtime", self.runtime_value)
        form.addRow("Port", self.port_value)
        form.addRow("Config", self.config_value)
        form.addRow("Cartesian", self.cartesian_value)
        form.addRow("Last Update", self.last_update_value)
        center_layout.addLayout(form)

        speed_row = QHBoxLayout()
        self.speed_label = QLabel("Speed")
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(50)
        self.speed_value = QLabel("50%")
        self.speed_slider.valueChanged.connect(self._on_speed_changed)
        speed_row.addWidget(self.speed_label)
        speed_row.addWidget(self.speed_slider, stretch=1)
        speed_row.addWidget(self.speed_value)
        center_layout.addLayout(speed_row)

        self.state_block = QLabel("Present: --\nTarget: --")
        self.state_block.setObjectName("monoBlock")
        self.state_block.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.state_block.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.state_block.setWordWrap(True)
        center_layout.addWidget(self.state_block, stretch=1)
        top.addWidget(self.center_group, stretch=1)

        self.right_group = QGroupBox("Joint Control")
        right_layout = QVBoxLayout(self.right_group)

        self.joint_rows_host = QWidget()
        self.joint_rows_layout = QVBoxLayout(self.joint_rows_host)
        self.joint_rows_layout.setContentsMargins(0, 0, 0, 0)
        self.joint_rows_layout.setSpacing(8)
        right_layout.addWidget(self.joint_rows_host)

        self.pose_group = QGroupBox("TCP (mm)")
        pose_form = QFormLayout(self.pose_group)
        self.pose_labels = {}
        for key in ("X", "Y", "Z"):
            value = QLabel("--")
            value.setObjectName("valueLabel")
            pose_form.addRow(key, value)
            self.pose_labels[key] = value
        right_layout.addWidget(self.pose_group)
        top.addWidget(self.right_group, stretch=1)

        bottom = QHBoxLayout()
        self.home_btn = QPushButton("Home")
        self.home_btn.setObjectName("primaryBtn")
        self.home_btn.clicked.connect(self.home_clicked.emit)
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_clicked.emit)
        self.status_light = QLabel("●")
        self.status_light.setStyleSheet("color:#10B981; font-size:16px;")
        bottom.addWidget(self.home_btn)
        bottom.addWidget(self.refresh_btn)
        bottom.addStretch()
        bottom.addWidget(self.status_light)
        root.addLayout(bottom)

        self.set_joint_names(())
        self._refresh_motion_controls()

    def _build_translation_pad(self) -> QWidget:
        pad = QFrame()
        grid = QGridLayout(pad)
        grid.setContentsMargins(6, 6, 6, 6)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)

        self.jog_buttons = {}
        up_btn = self._make_jog_button("+Z", "Z+")
        down_btn = self._make_jog_button("-Z", "Z-")
        forward_btn = self._make_jog_button("+X", "X+")
        back_btn = self._make_jog_button("-X", "X-")
        left_btn = self._make_jog_button("+Y", "Y+")
        right_btn = self._make_jog_button("-Y", "Y-")

        center = QFrame()
        center.setObjectName("jogPadCenter")
        center_grid = QGridLayout(center)
        center_grid.setContentsMargins(10, 10, 10, 10)
        center_grid.setHorizontalSpacing(8)
        center_grid.setVerticalSpacing(8)
        center_grid.addWidget(forward_btn, 0, 1, alignment=Qt.AlignCenter)
        center_grid.addWidget(left_btn, 1, 0, alignment=Qt.AlignCenter)
        center_grid.addWidget(right_btn, 1, 2, alignment=Qt.AlignCenter)
        center_grid.addWidget(back_btn, 2, 1, alignment=Qt.AlignCenter)

        grid.addWidget(QLabel("Up (+Z)"), 0, 0)
        grid.addWidget(up_btn, 0, 1, alignment=Qt.AlignCenter)
        grid.addWidget(down_btn, 0, 2, alignment=Qt.AlignCenter)
        grid.addWidget(QLabel("Down (-Z)"), 0, 3)
        grid.addWidget(center, 1, 1, 2, 2)
        grid.addWidget(QLabel("Left (+Y)"), 1, 0)
        grid.addWidget(QLabel("Right (-Y)"), 1, 3)
        grid.addWidget(QLabel("Forward (+X)"), 2, 3)
        grid.addWidget(QLabel("Back (-X)"), 2, 0)
        return pad

    def _make_jog_button(self, key: str, text: str) -> QPushButton:
        btn = QPushButton(text)
        btn.setObjectName("jogBtn")
        btn.pressed.connect(partial(self.cartesian_jog_pressed.emit, key))
        btn.released.connect(self.cartesian_jog_released.emit)
        self.jog_buttons[key] = btn
        return btn

    def _clear_layout(self, layout: QVBoxLayout):
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

    def set_joint_names(self, joint_names: Sequence[str]):
        next_joint_names = list(joint_names)
        if next_joint_names == self._joint_names:
            return
        self._joint_names = next_joint_names
        self._joint_rows.clear()
        self._clear_layout(self.joint_rows_layout)

        if not self._joint_names:
            placeholder = QLabel("Connect to the robot to load joint controls.")
            placeholder.setObjectName("captionLabel")
            placeholder.setWordWrap(True)
            self.joint_rows_layout.addWidget(placeholder)
            self.joint_rows_layout.addStretch()
            return

        for joint_name in self._joint_names:
            row_widget = QWidget()
            row = QHBoxLayout(row_widget)
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(8)
            name_label = QLabel(joint_name)
            value_label = QLabel("--")
            value_label.setObjectName("valueLabel")
            minus_btn = QPushButton("-")
            plus_btn = QPushButton("+")
            minus_btn.clicked.connect(partial(self.joint_step_requested.emit, joint_name, -1.0))
            plus_btn.clicked.connect(partial(self.joint_step_requested.emit, joint_name, 1.0))
            row.addWidget(name_label)
            row.addWidget(minus_btn)
            row.addWidget(value_label, stretch=1)
            row.addWidget(plus_btn)
            self.joint_rows_layout.addWidget(row_widget)
            self._joint_rows[joint_name] = (name_label, value_label, minus_btn, plus_btn)

        self.joint_rows_layout.addStretch()
        self._refresh_motion_controls()

    def set_joint_values(self, joint_values_deg: Mapping[str, float]):
        for joint_name, (_name_label, value_label, _minus_btn, _plus_btn) in self._joint_rows.items():
            value = joint_values_deg.get(joint_name)
            value_label.setText("--" if value is None else f"{value:.2f} deg")

    def set_pose_values_mm(self, pose_xyz_mm: Mapping[str, float] | None):
        if pose_xyz_mm is None:
            for label in self.pose_labels.values():
                label.setText("--")
            return
        for key in ("X", "Y", "Z"):
            value = pose_xyz_mm.get(key)
            self.pose_labels[key].setText("--" if value is None else f"{value:.1f}")

    def set_runtime_state(
        self,
        *,
        runtime: str,
        port: str,
        config_path: str,
        cartesian: str,
        last_update: str,
        present_summary: str,
        target_summary: str,
    ):
        self.runtime_value.setText(runtime)
        self.port_value.setText(port)
        self.config_value.setText(config_path)
        self.cartesian_value.setText(cartesian)
        self.last_update_value.setText(last_update)
        self.state_block.setText(f"Present: {present_summary}\nTarget: {target_summary}")

    def set_motion_enabled(self, enabled: bool):
        self._motion_enabled = bool(enabled)
        self._refresh_motion_controls()

    def set_cartesian_enabled(self, enabled: bool):
        self._cartesian_enabled = bool(enabled)
        self._refresh_motion_controls()

    def _refresh_motion_controls(self):
        motion_enabled = bool(self._motion_enabled)
        cartesian_enabled = bool(motion_enabled and self._cartesian_enabled)

        for _name_label, _value_label, minus_btn, plus_btn in self._joint_rows.values():
            minus_btn.setEnabled(motion_enabled)
            plus_btn.setEnabled(motion_enabled)

        for btn in self.jog_buttons.values():
            btn.setEnabled(cartesian_enabled)

        self.step_mode_combo.setEnabled(cartesian_enabled)
        self.step_dist_spin.setEnabled(cartesian_enabled)
        self.step_angle_spin.setEnabled(motion_enabled)
        self.home_btn.setEnabled(motion_enabled)
        self.refresh_btn.setEnabled(True)

    def set_status_light(self, level: str):
        color = {
            "normal": "#10B981",
            "warning": "#F59E0B",
            "fault": "#EF4444",
        }.get(level, "#10B981")
        self.status_light.setStyleSheet(f"color:{color}; font-size:16px;")

    def _on_speed_changed(self, value: int):
        self.speed_value.setText(f"{value}%")
        self.speed_changed.emit(value)
