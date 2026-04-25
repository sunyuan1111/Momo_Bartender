# Software AI Runtime

This folder keeps the local AI stack isolated from the robot-control package.

It contains:

- `emotion_cli.py`: facial emotion recognition via `EmotiEffLib`
- `stt_cli.py`: speech-to-text via `faster-whisper`
- `tts_cli.py`: text-to-speech via `Fun-CosyVoice3-0.5B-2512`
- `verify_stack.py`: end-to-end smoke test
- `setup_ai_env.sh`: one-shot environment bootstrap for `momo-ai`
- `runs/`: generated audio / text / json verification outputs

## Install And Verify

```bash
bash Software/ai/setup_ai_env.sh
```

That script:

- creates `conda` env `momo-ai` with Python 3.10
- clones `CosyVoice` under `Software/vendor/`
- installs the runtime dependencies for emotion / STT / TTS
- installs a CUDA 12.8 PyTorch stack suitable for RTX 5060 Laptop GPU
- downloads `Fun-CosyVoice3-0.5B-2512` on demand
- runs an end-to-end verification pass

For a lightweight runtime probe without re-running synthesis:

```bash
/home/yuan/miniconda3/envs/momo-ai/bin/python Software/ai/check_runtime.py
```

## Prefetch STT Models

Recommended local faster-whisper presets for this project:

- `tiny`: lowest latency fallback
- `small`: default practical choice
- `medium`: higher accuracy option

Prefetch the recommended set into fixed local directories under `Software/models/faster-whisper/`:

```bash
/home/yuan/miniconda3/envs/momo-ai/bin/python Software/ai/download_models.py \
  --stt-models tiny small medium
```

If you want every common preset that this repo currently supports:

```bash
/home/yuan/miniconda3/envs/momo-ai/bin/python Software/ai/download_models.py \
  --all-common-stt
```

## Runtime Commands

TTS:

```bash
/home/yuan/miniconda3/envs/momo-ai/bin/python Software/ai/tts_cli.py \
  --text "你好，我是 Momo，现在做语音输出联调。"
```

STT:

```bash
/home/yuan/miniconda3/envs/momo-ai/bin/python Software/ai/stt_cli.py \
  --audio Software/ai/runs/tts/latest.wav \
  --model small \
  --language zh
```

Emotion:

```bash
/home/yuan/miniconda3/envs/momo-ai/bin/python Software/ai/emotion_cli.py --demo-image
```

Realtime camera emotion recognition:

```bash
/home/yuan/miniconda3/envs/momo-ai/bin/python Software/ai/emotion_cli.py \
  --camera 0
```

Headless camera test without opening a preview window:

```bash
/home/yuan/miniconda3/envs/momo-ai/bin/python Software/ai/emotion_cli.py \
  --camera 0 \
  --no-display \
  --max-frames 12
```

Full verification:

```bash
/home/yuan/miniconda3/envs/momo-ai/bin/python Software/ai/verify_stack.py
```

Verification artifacts are written under `Software/ai/runs/verify/`.

## GPU Note

The scripts automatically switch to CUDA when `torch.cuda.is_available()` becomes `True`.

This runtime has been validated on Ubuntu with `nvidia-smi` working and:

- `Fun-CosyVoice3-0.5B-2512` on `cuda`
- `faster-whisper` on `cuda` with `float16`
- `EmotiEffLib` on `cuda`

The Python entrypoints also preload the bundled NVIDIA CUDA/cuDNN libraries from the virtual environment, so you do not need to export `LD_LIBRARY_PATH` manually.

On an 8 GB laptop GPU, avoid launching multiple TTS jobs in parallel. A single `CosyVoice3` process fits, but two concurrent TTS runs can exhaust VRAM.
