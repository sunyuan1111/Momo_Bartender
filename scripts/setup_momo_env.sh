#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONDA_BIN="${CONDA_BIN:-$HOME/miniconda3/bin/conda}"
ENV_NAME="${1:-momo}"
PYTHON_BIN="$HOME/miniconda3/envs/$ENV_NAME/bin/python"
PIP_INDEX_URL="${PIP_INDEX_URL:-https://pypi.org/simple}"

if [[ ! -x "$CONDA_BIN" ]]; then
  echo "conda not found at $CONDA_BIN" >&2
  exit 1
fi

"$CONDA_BIN" env remove -y -n "$ENV_NAME" >/dev/null 2>&1 || true
"$CONDA_BIN" create -y -n "$ENV_NAME" --override-channels -c conda-forge python=3.12 pip

PIP_INDEX_URL="$PIP_INDEX_URL" "$PYTHON_BIN" -m pip install --no-deps "lerobot==0.5.1"
PIP_INDEX_URL="$PIP_INDEX_URL" "$PYTHON_BIN" -m pip install \
  "numpy>=2.0,<2.3" \
  "draccus==0.10.0" \
  "pyserial>=3.5,<4" \
  "deepdiff>=7.0.1,<9" \
  "pybullet>=3.2,<4" \
  "tqdm>=4.64" \
  "feetech-servo-sdk>=1.0.0,<2.0.0"
PIP_INDEX_URL="$PIP_INDEX_URL" "$PYTHON_BIN" -m pip install --no-deps -e "$REPO_ROOT"

echo
echo "Environment '$ENV_NAME' is ready."
echo "Activate it with: conda activate $ENV_NAME"
