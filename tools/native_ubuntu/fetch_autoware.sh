#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ref_root="${ATLAS_AUTOWARE_ROOT:-$(cd "$repo_root/.." && pwd)/references/autoware}"
export ATLAS_AUTOWARE_ROOT="$ref_root"

if [[ ! -d "$ref_root/.git" ]]; then
  if [[ -e "$ref_root" ]]; then
    echo "Refusing to use non-Git Autoware path: $ref_root" >&2
    exit 2
  fi
  mkdir -p "$(dirname "$ref_root")"
  git clone --branch 1.9.0 --depth 1 https://github.com/autowarefoundation/autoware.git "$ref_root"
fi

expected_root="10718787ba6e28f038a0cb29ff99cc627b5abfd2"
actual_root="$(git -C "$ref_root" rev-parse HEAD)"
if [[ "$actual_root" != "$expected_root" ]]; then
  echo "Autoware root revision mismatch: $actual_root (expected $expected_root)." >&2
  exit 3
fi

if ! command -v vcs >/dev/null 2>&1; then
  echo "Missing vcstool. Install python3-vcstool, then rerun this script." >&2
  exit 4
fi
vcs import "$ref_root/src" < "$ref_root/repositories/autoware.repos"

while IFS=$'\t' read -r relative expected; do
  [[ "$relative" == relative_checkout || -z "$relative" ]] && continue
  checkout="$ref_root"
  [[ "$relative" == . ]] || checkout="$ref_root/$relative"
  if [[ ! -d "$checkout/.git" ]]; then
    echo "Pinned checkout missing: $checkout" >&2
    exit 5
  fi
  actual="$(git -C "$checkout" rev-parse HEAD)"
  if [[ "$actual" != "$expected" ]]; then
    echo "Pinned revision mismatch for $relative: $actual (expected $expected)." >&2
    exit 6
  fi
done < "$repo_root/tools/native_ubuntu/autoware_pins.tsv"

printf 'Pinned Autoware source verified at %s\n' "$ref_root"
