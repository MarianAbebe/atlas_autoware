#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
source src/atlas_full_stack_demo/scripts/environment.sh
python3 src/atlas_full_stack_demo/scripts/run_validation.py "$@"
