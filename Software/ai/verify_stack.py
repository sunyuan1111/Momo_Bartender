#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path

from common import RUNS_ROOT, save_demo_face, write_json


def main() -> int:
    import emotion_cli
    import stt_cli
    import tts_cli

    verify_dir = RUNS_ROOT / "verify"
    verify_dir.mkdir(parents=True, exist_ok=True)

    tts_audio = verify_dir / "cosyvoice_verify.wav"
    tts_json = verify_dir / "cosyvoice_verify.json"
    stt_json = verify_dir / "faster_whisper_verify.json"
    stt_txt = verify_dir / "faster_whisper_verify.txt"
    emotion_json = verify_dir / "emotion_verify.json"
    demo_face = save_demo_face(verify_dir / "demo_face.jpg")

    tts_cli.main = tts_cli.main  # keep linters quiet for direct imports
    stt_cli.main = stt_cli.main
    emotion_cli.main = emotion_cli.main

    import sys

    argv_backup = sys.argv[:]
    try:
        sys.argv = ["tts_cli.py", "--output", str(tts_audio), "--output-json", str(tts_json)]
        tts_cli.main()

        sys.argv = [
            "stt_cli.py",
            "--audio",
            str(tts_audio),
            "--output-json",
            str(stt_json),
            "--output-txt",
            str(stt_txt),
        ]
        stt_cli.main()

        sys.argv = [
            "emotion_cli.py",
            "--image",
            str(demo_face),
            "--output-json",
            str(emotion_json),
        ]
        emotion_cli.main()
    finally:
        sys.argv = argv_backup

    summary = {
        "tts_audio": str(tts_audio),
        "tts_json": str(tts_json),
        "stt_json": str(stt_json),
        "stt_txt": str(stt_txt),
        "emotion_json": str(emotion_json),
        "demo_face": str(demo_face),
    }
    summary_path = verify_dir / "summary.json"
    write_json(summary_path, summary)
    print(f"summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
