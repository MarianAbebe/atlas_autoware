#!/usr/bin/env bash
set -eo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$repo_root"
source src/atlas_full_stack_demo/scripts/environment.sh
export ROS_DOMAIN_ID=79
python3 src/atlas_closed_loop_demo/scripts/runtime_preflight.py "$@"
python3 src/atlas_closed_loop_demo/scripts/run_validation.py "$@"
