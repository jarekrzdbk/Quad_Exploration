#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
venv_dir=${QUAD_VENV:-"$repo_dir/.venv"}
if [[ ! -x "$venv_dir/bin/python" ]]; then
    echo "Run $repo_dir/install_local.sh first (use --vispy for OpenGL)." >&2
    exit 1
fi
if [[ "${1:-}" == --vispy ]]; then
    shift
    exec "$venv_dir/bin/python" "$repo_dir/Simulation/run_3D_simulation.py" "$@"
fi
exec "$venv_dir/bin/python" "$repo_dir/Simulation/run_local.py" "$@"
