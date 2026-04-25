#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

from common import (
    DEFAULT_STT_MODEL,
    FASTER_WHISPER_ROOT,
    RUNS_ROOT,
    FASTER_WHISPER_REPOS,
    faster_whisper_compute_type,
    faster_whisper_model_dir,
    preload_cuda_user_libs,
    torch_device,
    write_json,
)


def _resolve_local_model(model: str) -> str:
    direct_path = Path(model).expanduser()
    if direct_path.exists():
        return str(direct_path)

    if model in FASTER_WHISPER_REPOS:
        stable_dir = faster_whisper_model_dir(model)
        required_files = (
            stable_dir / "config.json",
            stable_dir / "model.bin",
            stable_dir / "tokenizer.json",
            stable_dir / "vocabulary.txt",
        )
        if all(path.exists() for path in required_files):
            return str(stable_dir)

    repo_root = FASTER_WHISPER_ROOT / f"models--Systran--faster-whisper-{model}"
    refs_main = repo_root / "refs" / "main"
    if not refs_main.exists():
        return model

    revision = refs_main.read_text(encoding="utf-8").strip()
    snapshot_dir = repo_root / "snapshots" / revision
    required_files = (
        snapshot_dir / "config.json",
        snapshot_dir / "model.bin",
        snapshot_dir / "tokenizer.json",
        snapshot_dir / "vocabulary.txt",
    )
    if all(path.exists() for path in required_files):
        return str(snapshot_dir)

    return model


def main() -> int:
    parser = argparse.ArgumentParser(description="Run local STT with faster-whisper.")
    parser.add_argument("--audio", type=Path, required=True, help="Input audio path.")
    parser.add_argument("--model", default=DEFAULT_STT_MODEL, help="Whisper model size or repo id.")
    parser.add_argument("--language", default="zh", help="Whisper language hint, e.g. zh/en.")
    parser.add_argument("--beam-size", type=int, default=5)
    parser.add_argument("--vad-filter", action="store_true", default=True)
    parser.add_argument("--no-vad-filter", dest="vad_filter", action="store_false")
    parser.add_argument("--output-json", type=Path, default=RUNS_ROOT / "stt" / "latest.json")
    parser.add_argument("--output-txt", type=Path, default=RUNS_ROOT / "stt" / "latest.txt")
    args = parser.parse_args()

    if not args.audio.exists():
        raise SystemExit(f"Audio not found: {args.audio}")

    preload_cuda_user_libs()
    device = torch_device()
    compute_type = faster_whisper_compute_type(device)
    model_ref = _resolve_local_model(args.model)

    from faster_whisper import WhisperModel

    model = WhisperModel(
        model_ref,
        device=device,
        compute_type=compute_type,
        download_root=str(FASTER_WHISPER_ROOT),
    )
    segments, info = model.transcribe(
        str(args.audio),
        language=args.language,
        beam_size=args.beam_size,
        vad_filter=args.vad_filter,
    )
    segments = list(segments)
    text = "".join(segment.text for segment in segments).strip()

    payload = {
        "audio": str(args.audio),
        "model": args.model,
        "model_ref": model_ref,
        "device": device,
        "compute_type": compute_type,
        "language": args.language,
        "duration": getattr(info, "duration", None),
        "duration_after_vad": getattr(info, "duration_after_vad", None),
        "text": text,
        "segments": [
            {
                "id": segment.id,
                "start": segment.start,
                "end": segment.end,
                "text": segment.text,
                "avg_logprob": getattr(segment, "avg_logprob", None),
            }
            for segment in segments
        ],
    }
    write_json(args.output_json, payload)
    args.output_txt.parent.mkdir(parents=True, exist_ok=True)
    args.output_txt.write_text(text + "\n", encoding="utf-8")

    print(f"audio: {args.audio}")
    print(f"device: {device}")
    print(f"compute_type: {compute_type}")
    print(f"text: {text}")
    print(f"json: {args.output_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
