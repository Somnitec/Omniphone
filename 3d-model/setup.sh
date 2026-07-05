#!/usr/bin/env bash
# Recreate the local Python venv for the CAD scripts on any machine.
# Usage:  ./setup.sh        (run from anywhere; it cd's to its own folder)
set -euo pipefail
cd "$(dirname "$0")"

PY="${PYTHON:-python3}"

echo "Creating .venv with $($PY --version)…"
"$PY" -m venv .venv
./.venv/bin/pip install --upgrade pip >/dev/null
echo "Installing deps (build123d pulls OpenCASCADE, ~few hundred MB the first time)…"
./.venv/bin/pip install -r requirements.txt

echo
echo "Done. Interpreter: 3d-model/.venv/bin/python"
echo "In VS Code: reload the window — .vscode/settings.json already points 'Run Python File' here."
