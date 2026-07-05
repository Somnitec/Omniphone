#!/usr/bin/env bash
# Recreate the local Python venv for the CAD scripts on any machine.
# Usage:  ./setup.sh        (run from anywhere; it cd's to its own folder)
#
# A venv is bound to one Python minor version. If the system Python is upgraded
# (e.g. 3.14 -> 3.15) the old .venv stops importing OpenCASCADE — just re-run this
# script and it rebuilds .venv against the current interpreter.
set -euo pipefail
cd "$(dirname "$0")"

PY="${PYTHON:-python3}"

echo "Creating .venv with $($PY --version)…"
rm -rf .venv                       # drop any stale env (e.g. built against an older Python)
"$PY" -m venv .venv
./.venv/bin/pip install --upgrade pip >/dev/null
echo "Installing deps (build123d pulls OpenCASCADE, ~few hundred MB the first time)…"
./.venv/bin/pip install --only-binary :all: -r requirements.txt

echo "Verifying the OpenCASCADE bindings import…"
./.venv/bin/python -c "import build123d, OCP; print('  build123d', build123d.__version__, 'OK')"

echo
echo "Done. Interpreter: 3d-model/.venv/bin/python"
echo "In VS Code: reload the window — the repo .vscode/settings.json already points the"
echo "Python interpreter and the 'Run Python File' button at this venv."
