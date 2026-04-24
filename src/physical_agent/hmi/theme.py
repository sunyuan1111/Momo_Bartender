"""Minimal stylesheet helpers for the Quick Move GUI."""

from __future__ import annotations


def get_stylesheet() -> str:
    return """
QMainWindow, QWidget {
    background-color: #F2F4F7;
    color: #1E2430;
    font-family: "Segoe UI", "Microsoft YaHei", sans-serif;
    font-size: 14px;
}

QFrame, QGroupBox {
    background-color: #FFFFFF;
}

QGroupBox {
    border: 1px solid rgba(0, 0, 0, 0.08);
    border-radius: 14px;
    margin-top: 14px;
    padding: 14px 14px 12px 14px;
    font-weight: 600;
}

QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
    color: #5D6572;
    background: transparent;
}

QLineEdit, QComboBox, QDoubleSpinBox {
    min-height: 42px;
    border-radius: 10px;
    border: 1px solid rgba(0, 0, 0, 0.08);
    background-color: #FFFFFF;
    color: #1E2430;
    padding: 0 12px;
}

QPushButton {
    min-height: 42px;
    border-radius: 10px;
    border: 1px solid rgba(0, 0, 0, 0.08);
    background: #F6F7F9;
    color: #1E2430;
    font-weight: 600;
    padding: 0 14px;
}

QPushButton:hover {
    background: #FFFFFF;
}

QPushButton:pressed {
    background: #EEF1F5;
}

QPushButton:disabled {
    color: #8A93A3;
    background: #F6F7F9;
    border-color: rgba(0, 0, 0, 0.05);
}

QPushButton#primaryBtn {
    background: #3748CA;
    border-color: #3748CA;
    color: #FFFFFF;
}

QPushButton#primaryBtn:hover {
    background: #4A59D8;
    border-color: #4A59D8;
}

QPushButton#primaryBtn:pressed {
    background: #2D3DB2;
    border-color: #2D3DB2;
}

QPushButton#jogBtn {
    min-width: 64px;
    max-width: 64px;
    min-height: 64px;
    max-height: 64px;
    border-radius: 14px;
    font-size: 16px;
}

QFrame#jogPadCenter {
    border: 1px solid rgba(0, 0, 0, 0.08);
    border-radius: 18px;
    background: #F6F7F9;
}

QFrame#globalStatusBar {
    border: 1px solid rgba(0, 0, 0, 0.05);
    border-radius: 12px;
    background: #F7F8FA;
}

QFrame#globalStatusBar QLabel {
    color: #5D6572;
    font-weight: 600;
}

QLabel#captionLabel {
    color: #5D6572;
}

QLabel#valueLabel {
    color: #1E2430;
    font-weight: 700;
}

QLabel#monoBlock {
    border: 1px solid rgba(0, 0, 0, 0.06);
    border-radius: 10px;
    background: #F8FAFC;
    padding: 10px;
    font-family: "Cascadia Mono", "Consolas", monospace;
    font-size: 12px;
}

QSlider::groove:horizontal {
    height: 6px;
    border-radius: 3px;
    background: #D7DCE5;
}

QSlider::handle:horizontal {
    width: 18px;
    margin: -6px 0;
    border-radius: 9px;
    background: #3748CA;
}
"""
