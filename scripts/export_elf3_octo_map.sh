#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/../src/bxi_example_nav3d/scripts/export_elf3_octo_map.sh" "$@"
