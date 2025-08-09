#!/bin/bash
set -e

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <submodules_file> <submodules_action>"
    exit 1
fi

SUBMODULES_FILE=$1
SUBMODULES_ACTION=$2

if [ ! -f "$SUBMODULES_FILE" ]; then
    echo "File $SUBMODULES_FILE not found"
    exit 1
fi

while read -r SUBMODULE_URL SUBMODULE_PATH; do
    [[ -z "$SUBMODULE_PATH" || "$SUBMODULE_PATH" =~ ^# ]] && continue
    ./scripts/submodule.sh "$SUBMODULE_URL" "$SUBMODULE_PATH" "$SUBMODULES_ACTION"
done < "$SUBMODULES_FILE"
