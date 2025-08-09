#!/bin/bash
set -e

if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <submodule_url> <submodule_path> <submodule_action>"
    exit 1
fi

SUBMODULE_URL="$1"
SUBMODULE_PATH="$2"
SUBMODULE_ACTION="$3"

REPO_ROOT=$(git rev-parse --show-toplevel)
if [[ "$SUBMODULE_PATH" = /* ]]; then
    SUBMODULE_PATH=$(realpath --relative-to="$REPO_ROOT" "$SUBMODULE_PATH")
fi

function add_submodule {
    if [ -d "$SUBMODULE_PATH" ]; then
        echo "Submodule $SUBMODULE_PATH already exists at $SUBMODULE_PATH" 
        return
    fi

    echo "Adding submodule $SUBMODULE_PATH → $SUBMODULE_PATH"
    git submodule add -f "$SUBMODULE_URL" "$SUBMODULE_PATH"
}

function remove_submodule {
    if [ ! -d "$SUBMODULE_PATH" ]; then
        echo "Submodule $SUBMODULE_PATH doesn't exist at $SUBMODULE_PATH"        
        return
    fi

    echo "Removing submodule $SUBMODULE_PATH"
    git submodule deinit -f "$SUBMODULE_PATH"
    git rm -rf "$SUBMODULE_PATH"
    rm -rf ".git/modules/$SUBMODULE_PATH"
    rm -rf "$SUBMODULE_PATH"
}

function update_submodule {
    if [ ! -d "$SUBMODULE_PATH" ]; then
        echo "Submodule $SUBMODULE_PATH doesn't exist at $SUBMODULE_PATH"
        return
    fi

    echo "Updating submodule $SUBMODULE_PATH"
    git submodule update --init --remote --recursive "$SUBMODULE_PATH"
}

case "$SUBMODULE_ACTION" in
    add) add_submodule ;;
    remove) remove_submodule ;;
    update) update_submodule ;;
    *) echo "Invalid action: $SUBMODULE_ACTION"; exit 1 ;;
esac
