#!/usr/bin/env bash
# One-time setup for the AI node's Python environment.
# Creates a venv next to this workspace that can still see rclpy/cv_bridge
# (via --system-site-packages) while getting its own numpy/torch/ultralytics
# versions that would otherwise clash with the ones ROS ships.
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VENV_DIR="${WS_DIR}/yolo_venv"

python3 -m venv --system-site-packages "${VENV_DIR}"
source "${VENV_DIR}/bin/activate"
pip install --upgrade pip
pip install -r "$(dirname "${BASH_SOURCE[0]}")/requirements.txt"

echo "Done. Venv created at ${VENV_DIR}"
