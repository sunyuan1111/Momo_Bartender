#!/usr/bin/env python3

from __future__ import annotations

import importlib
import os
import platform
import subprocess
import sys
from pathlib import Path

from common import add_vendor_paths, cuda_ready, ensure_runtime_dirs, nvidia_smi_path, preload_cuda_user_libs


def _check_module(name: str) -> str:
    try:
        module = importlib.import_module(name)
    except Exception as exc:
        return f"{name}: missing ({exc})"
    version = getattr(module, "__version__", None)
    return f"{name}: ok ({version})" if version else f"{name}: ok"


def main() -> int:
    ensure_runtime_dirs()
    add_vendor_paths()
    preload_cuda_user_libs()

    print("System")
    print("------")
    print(f"python: {sys.version.split()[0]}")
    print(f"platform: {platform.platform()}")
    print(f"cwd: {Path.cwd()}")

    print("\nPython Packages")
    print("---------------")
    for name in (
        "torch",
        "torchaudio",
        "cv2",
        "emotiefflib",
        "faster_whisper",
        "whisper",
        "modelscope",
        "onnxruntime",
        "transformers",
    ):
        print(_check_module(name))

    try:
        import torch

        print(f"torch.cuda.is_available: {torch.cuda.is_available()}")
        print(f"torch.cuda.device_count: {torch.cuda.device_count()}")
        if torch.cuda.is_available():
            for idx in range(torch.cuda.device_count()):
                print(f"torch.cuda.device[{idx}]: {torch.cuda.get_device_name(idx)}")
    except Exception:
        pass

    try:
        import onnxruntime as ort

        print(f"onnxruntime.providers: {', '.join(ort.get_available_providers())}")
    except Exception:
        pass

    print("\nGPU")
    print("---")
    print(f"cuda_ready: {cuda_ready()}")
    smi = nvidia_smi_path()
    if smi is None:
        print("nvidia-smi: not found in PATH")
    else:
        try:
            result = subprocess.run(
                [smi, "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"],
                check=True,
                capture_output=True,
                text=True,
            )
            print("nvidia-smi:")
            for line in result.stdout.splitlines():
                if line.strip():
                    print(f"  {line.strip()}")
        except Exception as exc:
            print(f"nvidia-smi: unavailable ({exc})")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
