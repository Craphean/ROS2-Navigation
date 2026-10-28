#!/usr/bin/env bash
# Convenience wrapper: start the mine inspection console from the repo root.
#   ./scripts/console.sh
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/console/run.sh" "$@"
