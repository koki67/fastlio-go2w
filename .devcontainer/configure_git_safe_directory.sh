#!/bin/bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! command -v git >/dev/null 2>&1; then
    echo "git is not installed; skipping safe.directory setup." >&2
    exit 0
fi

add_safe_directory() {
    local path="$1"

    if [ ! -e "$path" ]; then
        return 0
    fi

    if git config --global --get-all safe.directory | grep -Fxq "$path"; then
        return 0
    fi

    git config --global --add safe.directory "$path"
}

add_submodules_from_gitmodules() {
    local gitmodules_file="$1"
    local base_dir

    base_dir="$(dirname "$gitmodules_file")"

    git config --file "$gitmodules_file" --get-regexp '^submodule\..*\.path$' 2>/dev/null \
        | awk '{print $2}' \
        | while read -r sub_path; do
            add_safe_directory "$base_dir/$sub_path"
        done
}

add_safe_directory "$REPO_ROOT"

find "$REPO_ROOT" -path "$REPO_ROOT/.git" -prune -o -name .gitmodules -type f -print 2>/dev/null \
    | while read -r gitmodules_file; do
        add_submodules_from_gitmodules "$gitmodules_file"
    done

echo "Configured git safe.directory for workspace."
