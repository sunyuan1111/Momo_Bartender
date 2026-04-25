#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

from common import (
    COSYVOICE_MODEL_DIR,
    DEFAULT_STT_PRELOAD_MODELS,
    FASTER_WHISPER_REPOS,
    faster_whisper_model_dir,
    ensure_runtime_dirs,
)


def ensure_cosyvoice3_model(local_dir: Path | None = None) -> Path:
    ensure_runtime_dirs()
    target = local_dir or COSYVOICE_MODEL_DIR
    required_files = [
        target / "cosyvoice3.yaml",
        target / "llm.pt",
        target / "flow.pt",
        target / "hift.pt",
        target / "campplus.onnx",
        target / "speech_tokenizer_v3.onnx",
        target / "CosyVoice-BlankEN" / "config.json",
        target / "CosyVoice-BlankEN" / "model.safetensors",
    ]
    if all(path.exists() for path in required_files):
        return target

    from modelscope import snapshot_download

    snapshot_download(
        "FunAudioLLM/Fun-CosyVoice3-0.5B-2512",
        local_dir=str(target),
        ignore_patterns=[
            "flow.decoder.estimator*",
            "llm.rl.pt",
            "speech_tokenizer_*.batch.onnx",
            "*.plan",
            "vllm/*",
        ],
    )
    return target


def ensure_faster_whisper_model(model: str, local_dir: Path | None = None) -> Path:
    ensure_runtime_dirs()
    if model not in FASTER_WHISPER_REPOS:
        raise ValueError(f"Unsupported faster-whisper preset: {model}")

    target = local_dir or faster_whisper_model_dir(model)
    required_files = [
        target / "config.json",
        target / "model.bin",
        target / "tokenizer.json",
        target / "vocabulary.txt",
    ]
    if all(path.exists() for path in required_files):
        return target

    from huggingface_hub import snapshot_download

    snapshot_download(
        repo_id=FASTER_WHISPER_REPOS[model],
        local_dir=str(target),
        allow_patterns=[
            "config.json",
            "model.bin",
            "tokenizer.json",
            "vocabulary.txt",
        ],
    )
    return target


def main() -> int:
    parser = argparse.ArgumentParser(description="Download Software-side AI models.")
    parser.add_argument(
        "--cosyvoice3-dir",
        type=Path,
        default=COSYVOICE_MODEL_DIR,
        help="Local path for Fun-CosyVoice3-0.5B-2512.",
    )
    parser.add_argument(
        "--stt-models",
        nargs="*",
        choices=sorted(FASTER_WHISPER_REPOS),
        help="Prefetch faster-whisper models into fixed local directories.",
    )
    parser.add_argument(
        "--all-common-stt",
        action="store_true",
        help="Prefetch tiny/base/small/medium/large-v3 for faster-whisper.",
    )
    args = parser.parse_args()

    if not args.stt_models and not args.all_common_stt:
        target = ensure_cosyvoice3_model(args.cosyvoice3_dir)
        print(target)
        return 0

    models = list(args.stt_models or [])
    if args.all_common_stt:
        for model in DEFAULT_STT_PRELOAD_MODELS:
            if model not in models:
                models.append(model)

    for model in models:
        target = ensure_faster_whisper_model(model)
        print(f"{model}: {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
