#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec python3 backend/app_launcher.py --host "${HOST:-127.0.0.1}" --port "${PORT:-8000}" "$@"
