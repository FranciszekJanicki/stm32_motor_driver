#!/bin/bash

if [ "$#" -lt 2 ]; then 
    echo "Usage: $0 <lint_action> <lint_dir>"
    exit 1 
fi

BUILD_DIR="build"
LINT_ACTION="$1"
LINT_DIR="$(realpath "$2")"

mapfile -d '' FILES < <(
    find "$LINT_DIR" -type f \( -iname "*.h" -o -iname "*.hpp" -o -iname "*.c" -o -iname "*.cpp" \) -print0
)

if [ "${#FILES[@]}" -eq 0 ]; then
    echo "No source files found in $LINT_DIR"
    exit 0
fi

function clang_tidy {
    clang-tidy -p "$BUILD_DIR" -quiet "${FILES[@]}" || true
}

function clang_format {
    clang-format -i "${FILES[@]}"
}

case "$LINT_ACTION" in
    tidy)  clang_tidy  ;;
    format) clang_format ;;
    *)      echo "Invalid action: $LINT_ACTION"; exit 1 ;;
esac
