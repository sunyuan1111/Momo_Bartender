from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


def _configure_qt_environment() -> None:
    conda_plugins_dir = Path(sys.prefix) / "plugins"
    conda_platforms_dir = conda_plugins_dir / "platforms"
    if conda_plugins_dir.is_dir():
        # Prefer the conda-managed Qt plugin tree when present. This avoids
        # mixing conda's Qt runtime with wheel-bundled PyQt5 plugins.
        os.environ["QT_PLUGIN_PATH"] = str(conda_plugins_dir)
    if conda_platforms_dir.is_dir():
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(conda_platforms_dir)

    os.environ.setdefault("QT_OPENGL", "software")
    os.environ.setdefault("QT_XCB_GL_INTEGRATION", "none")
    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Quick Move GUI for the STS3215 arm controller.")
    parser.add_argument(
        "--config",
        default="configs/arm2_sts3215.example.json",
        help="Path to the JSON arm config file.",
    )
    parser.add_argument(
        "--refresh-ms",
        type=int,
        default=500,
        help="Refresh interval for reading arm state.",
    )
    return parser


def main() -> None:
    _configure_qt_environment()
    parser = build_parser()
    args = parser.parse_args()

    try:
        from PyQt5.QtWidgets import QApplication
    except ImportError as exc:
        raise SystemExit(
            "PyQt5 is required for the GUI. Install it with `pip install PyQt5` or the project gui extra."
        ) from exc

    from physical_agent.hmi.window import QuickMoveWindow

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = QuickMoveWindow(
        config_path=Path(args.config).expanduser().resolve(),
        refresh_interval_ms=args.refresh_ms,
    )
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
