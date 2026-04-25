#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
AI_ROOT="$REPO_ROOT/Software/ai"
VENDOR_ROOT="$REPO_ROOT/Software/vendor"
COSYVOICE_ROOT="$VENDOR_ROOT/CosyVoice"
CONDA_BIN="${CONDA_BIN:-$HOME/miniconda3/bin/conda}"
ENV_NAME="${1:-momo-ai}"
PIP_INDEX_URL="${PIP_INDEX_URL:-https://pypi.org/simple}"
PYTORCH_INDEX_URL="${PYTORCH_INDEX_URL:-https://download.pytorch.org/whl/cu128}"
PYTHON_BIN="$HOME/miniconda3/envs/$ENV_NAME/bin/python"
PIP_BIN="$HOME/miniconda3/envs/$ENV_NAME/bin/pip"

if [[ ! -x "$CONDA_BIN" ]]; then
  echo "conda not found at $CONDA_BIN" >&2
  exit 1
fi

mkdir -p "$VENDOR_ROOT" "$REPO_ROOT/Software/models" "$REPO_ROOT/Software/ai/runs"

if [[ ! -d "$COSYVOICE_ROOT/.git" ]]; then
  git clone --recursive --depth 1 https://github.com/FunAudioLLM/CosyVoice.git "$COSYVOICE_ROOT"
else
  git -C "$COSYVOICE_ROOT" submodule update --init --recursive
fi

if [[ ! -x "$PYTHON_BIN" ]]; then
  "$CONDA_BIN" create -y -n "$ENV_NAME" python=3.10 pip
fi

PIP_INDEX_URL="$PYTORCH_INDEX_URL" "$PIP_BIN" install --upgrade --force-reinstall \
  "torch==2.7.1" \
  "torchvision==0.22.1" \
  "torchaudio==2.7.1"

PIP_INDEX_URL="$PIP_INDEX_URL" "$PIP_BIN" install \
  "numpy==1.26.4" \
  "fsspec==2024.12.0" \
  "packaging==24.2" \
  "protobuf==7.34.1" \
  "coloredlogs==15.0.1" \
  "humanfriendly==10.0" \
  "flatbuffers==25.12.19" \
  "onnx==1.16.0" \
  "transformers==4.51.3" \
  "tiktoken" \
  "ctranslate2==4.7.1" \
  "x-transformers==2.11.24" \
  "lightning==2.2.4" \
  "soundfile==0.12.1" \
  "librosa==0.10.2" \
  "matplotlib==3.7.5" \
  "pyarrow==18.1.0" \
  "pyworld==0.3.4" \
  "pydantic==2.7.0" \
  "inflect==7.3.1" \
  "wetext==0.0.4" \
  "hyperpyyaml==1.2.3" \
  "modelscope==1.20.0" \
  "conformer==0.3.2" \
  "diffusers==0.29.0" \
  "omegaconf==2.3.0" \
  "rich==13.7.1" \
  "opencv-python==4.10.0.84" \
  "regex" \
  "scipy" \
  "einops" \
  "gdown==5.1.0" \
  "wget==3.2" \
  "hydra-core==1.3.2" \
  "Unidecode" \
  "timm==0.9.16" \
  "av>=11" \
  "setuptools<81"

"$PIP_BIN" uninstall -y onnxruntime >/dev/null 2>&1 || true
PIP_INDEX_URL="$PIP_INDEX_URL" "$PIP_BIN" install --no-deps --force-reinstall \
  "onnxruntime-gpu==1.20.1"

PIP_INDEX_URL="$PIP_INDEX_URL" "$PIP_BIN" install --no-deps \
  "faster-whisper==1.2.1" \
  "emotiefflib==1.1.1"

PIP_INDEX_URL="$PIP_INDEX_URL" "$PIP_BIN" install --no-deps --force-reinstall \
  "openai-whisper==20231117"

"$PYTHON_BIN" "$AI_ROOT/check_runtime.py"
"$PYTHON_BIN" "$AI_ROOT/verify_stack.py"
