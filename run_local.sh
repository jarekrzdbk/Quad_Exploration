#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
venv_dir=${QUAD_VENV:-"$repo_dir/.venv"}
if [[ ! -x "$venv_dir/bin/python" ]]; then
    "${QUAD_PYTHON:-python3}" -m venv "$venv_dir"
fi
if ! cmp -s "$repo_dir/requirements.txt" "$venv_dir/.quad-requirements"; then
    "$venv_dir/bin/python" -m pip install -r "$repo_dir/requirements.txt"
    cp "$repo_dir/requirements.txt" "$venv_dir/.quad-requirements"
fi
exec "$venv_dir/bin/python" "$repo_dir/Simulation/run_local.py" "$@"
