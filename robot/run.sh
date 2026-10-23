#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec python3 backend/server.py --host "${HOST:-127.0.0.1}" --port "${PORT:-8000}" "$@"
