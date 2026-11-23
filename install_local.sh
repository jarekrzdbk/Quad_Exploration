#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
case "${1:-}" in
    "") manifest=requirements.txt ;;
    --vispy) manifest=requirements-vispy.txt ;;
    *) echo "Usage: $0 [--vispy]" >&2; exit 2 ;;
esac
if (( $# > 1 )); then echo "Usage: $0 [--vispy]" >&2; exit 2; fi
venv_dir=${QUAD_VENV:-"$repo_dir/.venv"}
if [[ ! -x "$venv_dir/bin/python" ]]; then
    "${QUAD_PYTHON:-python3}" -m venv "$venv_dir"
fi
"$venv_dir/bin/python" -m pip install -r "$repo_dir/$manifest"
