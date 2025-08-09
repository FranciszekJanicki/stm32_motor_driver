#!/bin/bash

if [ "$#" -lt 2 ]; then 
    echo "Usage: $0 <lint_action> <lint_dir> ..."
    exit 1 
fi

BUILD_DIR="build"
LINT_ACTION="$1"
shift

for LINT_DIR in "$@"; do
    ./scripts/linter.sh "$LINT_ACTION" "$LINT_DIR"
done
