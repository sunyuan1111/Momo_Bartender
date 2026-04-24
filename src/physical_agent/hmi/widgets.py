"""Reusable widgets for the Quick Move GUI."""

from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel


class GlobalStatusBar(QFrame):
    def __init__(self):
        super().__init__()
        self.setObjectName("globalStatusBar")
        self.setMinimumHeight(42)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(14)

        self.connection_label = QLabel("Connection: --")
        self.config_label = QLabel("Config: --")
        self.cartesian_label = QLabel("Cartesian: --")
        self.speed_label = QLabel("Speed: --")
        self.message_label = QLabel("Message: --")

        for label in (
            self.connection_label,
            self.config_label,
            self.cartesian_label,
            self.speed_label,
            self.message_label,
        ):
            label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
            layout.addWidget(label)

        layout.addStretch()

    def set_connection(self, text: str):
        self.connection_label.setText(text)

    def set_config(self, text: str):
        self.config_label.setText(text)

    def set_cartesian(self, text: str):
        self.cartesian_label.setText(text)

    def set_speed(self, text: str):
        self.speed_label.setText(text)

    def set_message(self, text: str):
        self.message_label.setText(text)
