#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import torch
import torchaudio

from common import (
    COSYVOICE_MODEL_DIR,
    DEFAULT_PROMPT_TEXT,
    DEFAULT_PROMPT_WAV,
    DEFAULT_TTS_INSTRUCT,
    DEFAULT_TTS_TEXT,
    RUNS_ROOT,
    add_vendor_paths,
    ensure_runtime_dirs,
    preload_cuda_user_libs,
    torch_device,
    write_json,
)
from download_models import ensure_cosyvoice3_model


def main() -> int:
    parser = argparse.ArgumentParser(description="Run local TTS with Fun-CosyVoice3-0.5B-2512.")
    parser.add_argument("--text", default=DEFAULT_TTS_TEXT, help="Target text to synthesize.")
    parser.add_argument(
        "--instruct",
        default=DEFAULT_TTS_INSTRUCT,
        help="CosyVoice3 instruction prompt ending with <|endofprompt|>.",
    )
    parser.add_argument("--prompt-wav", type=Path, default=DEFAULT_PROMPT_WAV)
    parser.add_argument("--prompt-text", default=DEFAULT_PROMPT_TEXT)
    parser.add_argument("--model-dir", type=Path, default=COSYVOICE_MODEL_DIR)
    parser.add_argument("--output", type=Path, default=RUNS_ROOT / "tts" / "latest.wav")
    parser.add_argument("--output-json", type=Path, default=RUNS_ROOT / "tts" / "latest.json")
    args = parser.parse_args()

    ensure_runtime_dirs()
    add_vendor_paths()
    preload_cuda_user_libs()
    model_dir = ensure_cosyvoice3_model(args.model_dir)

    if not args.prompt_wav.exists():
        raise SystemExit(f"Prompt wav not found: {args.prompt_wav}")

    from cosyvoice.cli.cosyvoice import AutoModel

    device = torch_device()
    cosyvoice = AutoModel(
        model_dir=str(model_dir),
        load_trt=False,
        load_vllm=False,
        fp16=(device == "cuda"),
    )

    chunks = []
    for item in cosyvoice.inference_instruct2(
        args.text,
        args.instruct,
        str(args.prompt_wav),
        stream=False,
    ):
        chunks.append(item["tts_speech"])
    if not chunks:
        raise SystemExit("CosyVoice returned no audio chunks")

    speech = torch.cat(chunks, dim=1)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torchaudio.save(str(args.output), speech, cosyvoice.sample_rate)

    payload = {
        "model_dir": str(model_dir),
        "device": device,
        "text": args.text,
        "instruct": args.instruct,
        "prompt_wav": str(args.prompt_wav),
        "sample_rate": cosyvoice.sample_rate,
        "output": str(args.output),
    }
    write_json(args.output_json, payload)

    print(f"output: {args.output}")
    print(f"device: {device}")
    print(f"json: {args.output_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
