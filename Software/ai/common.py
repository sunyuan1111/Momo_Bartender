#!/usr/bin/env python3

from __future__ import annotations

import json
import os
import shutil
import sys
import urllib.request
import ctypes
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
SOFTWARE_ROOT = REPO_ROOT / "Software"
AI_ROOT = SOFTWARE_ROOT / "ai"
VENDOR_ROOT = SOFTWARE_ROOT / "vendor"
MODELS_ROOT = SOFTWARE_ROOT / "models"
RUNS_ROOT = AI_ROOT / "runs"
DEMO_ROOT = AI_ROOT / "demo_assets"

COSYVOICE_ROOT = VENDOR_ROOT / "CosyVoice"
MATCHA_ROOT = COSYVOICE_ROOT / "third_party" / "Matcha-TTS"
COSYVOICE_MODEL_DIR = MODELS_ROOT / "cosyvoice" / "Fun-CosyVoice3-0.5B-2512"
FASTER_WHISPER_ROOT = MODELS_ROOT / "faster-whisper"
FASTER_WHISPER_REPOS = {
    "tiny": "Systran/faster-whisper-tiny",
    "base": "Systran/faster-whisper-base",
    "small": "Systran/faster-whisper-small",
    "medium": "Systran/faster-whisper-medium",
    "large-v3": "Systran/faster-whisper-large-v3",
}
DEFAULT_STT_PRELOAD_MODELS = ("tiny", "base", "small", "medium", "large-v3")

DEFAULT_PROMPT_WAV = COSYVOICE_ROOT / "asset" / "zero_shot_prompt.wav"
DEFAULT_PROMPT_TEXT = "You are a helpful assistant.<|endofprompt|>希望你以后能够做的比我还好呦。"
DEFAULT_TTS_TEXT = "你好，我是 Momo，现在正在做本地语音与情绪模型联调。"
DEFAULT_TTS_INSTRUCT = "You are a helpful assistant. 请用自然、清晰、友好的语气表达。<|endofprompt|>"
DEFAULT_STT_MODEL = "small"
DEFAULT_EMOTION_MODEL = "enet_b0_8_best_vgaf"
DEFAULT_EMOTION_ENGINE = "torch"
DEFAULT_DEMO_FACE_URL = "https://raw.githubusercontent.com/opencv/opencv/master/samples/data/lena.jpg"
_CUDA_LIBS_PRELOADED = False


def ensure_runtime_dirs() -> None:
    for path in (
        VENDOR_ROOT,
        MODELS_ROOT,
        RUNS_ROOT,
        DEMO_ROOT,
        MODELS_ROOT / "cosyvoice",
        FASTER_WHISPER_ROOT,
        RUNS_ROOT / "tts",
        RUNS_ROOT / "stt",
        RUNS_ROOT / "emotion",
        RUNS_ROOT / "verify",
    ):
        path.mkdir(parents=True, exist_ok=True)


def add_vendor_paths() -> None:
    for path in (COSYVOICE_ROOT, MATCHA_ROOT):
        path_str = str(path)
        if path.exists() and path_str not in sys.path:
            sys.path.insert(0, path_str)


def preload_cuda_user_libs() -> None:
    global _CUDA_LIBS_PRELOADED
    if _CUDA_LIBS_PRELOADED:
        return

    site_root = Path(sys.prefix) / "lib" / f"python{sys.version_info.major}.{sys.version_info.minor}" / "site-packages"
    nvidia_root = site_root / "nvidia"
    if not nvidia_root.exists():
        _CUDA_LIBS_PRELOADED = True
        return

    lib_dirs = [
        nvidia_root / "cuda_runtime" / "lib",
        nvidia_root / "cuda_nvrtc" / "lib",
        nvidia_root / "nvjitlink" / "lib",
        nvidia_root / "cublas" / "lib",
        nvidia_root / "cudnn" / "lib",
        nvidia_root / "cufft" / "lib",
        nvidia_root / "curand" / "lib",
        nvidia_root / "cusolver" / "lib",
        nvidia_root / "cusparse" / "lib",
        nvidia_root / "cusparselt" / "lib",
        nvidia_root / "nccl" / "lib",
        nvidia_root / "nvtx" / "lib",
    ]

    for lib_dir in lib_dirs:
        if not lib_dir.exists():
            continue
        for so_path in sorted(lib_dir.glob("*.so*")):
            try:
                ctypes.CDLL(str(so_path), mode=ctypes.RTLD_GLOBAL)
            except OSError:
                continue

    _CUDA_LIBS_PRELOADED = True


def torch_device(prefer_cuda: bool = True) -> str:
    try:
        import torch
    except Exception:
        return "cpu"
    if prefer_cuda and torch.cuda.is_available():
        return "cuda"
    return "cpu"


def cuda_ready() -> bool:
    return torch_device(prefer_cuda=True) == "cuda"


def faster_whisper_compute_type(device: str) -> str:
    return "float16" if device == "cuda" else "int8"


def faster_whisper_model_dir(model: str) -> Path:
    return FASTER_WHISPER_ROOT / model


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def save_demo_face(path: Path | None = None) -> Path:
    ensure_runtime_dirs()
    target = path or (DEMO_ROOT / "lena.jpg")
    if target.exists():
        return target
    urllib.request.urlretrieve(DEFAULT_DEMO_FACE_URL, target)
    return target


def nvidia_smi_path() -> str | None:
    return shutil.which("nvidia-smi")


def env_python(env_name: str = "momo-ai") -> Path:
    return Path.home() / "miniconda3" / "envs" / env_name / "bin" / "python"


def env_pip(env_name: str = "momo-ai") -> Path:
    return Path.home() / "miniconda3" / "envs" / env_name / "bin" / "pip"
