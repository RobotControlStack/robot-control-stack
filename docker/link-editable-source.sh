#!/bin/sh
set -eu

REPO_ROOT="${1:-/workspace/robot-control-stack}"

if [ ! -d "$REPO_ROOT" ]; then
    echo "Mounted repo not found at $REPO_ROOT; leaving installed packages unchanged."
    exit 0
fi

SITE_PACKAGES="$(python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"

link_mixed_package() {
    src_dir="$1"
    dst_dir="$2"
    keep_dir_name="${3:-}"

    if [ ! -d "$src_dir" ] || [ ! -d "$dst_dir" ]; then
        return
    fi

    # Replace only the Python sources from the mounted repo and keep compiled
    # artifacts that were installed into site-packages during image build.
    for path in "$src_dir"/* "$src_dir"/.[!.]* "$src_dir"/..?*; do
        [ -e "$path" ] || continue
        name="$(basename "$path")"
        if [ -n "$keep_dir_name" ] && [ "$name" = "$keep_dir_name" ]; then
            continue
        fi
        rm -rf "$dst_dir/$name"
        cp -as "$path" "$dst_dir/$name"
    done
}

link_pure_python_package() {
    src_dir="$1"
    dst_dir="$2"

    if [ ! -d "$src_dir" ]; then
        return
    fi

    rm -rf "$dst_dir"
    ln -s "$src_dir" "$dst_dir"
}

link_mixed_package "$REPO_ROOT/python/rcs" "$SITE_PACKAGES/rcs" "_core"
link_mixed_package "$REPO_ROOT/extensions/rcs_fr3/src/rcs_fr3" "$SITE_PACKAGES/rcs_fr3" "_core"
link_pure_python_package "$REPO_ROOT/extensions/rcs_realsense/src/rcs_realsense" "$SITE_PACKAGES/rcs_realsense"
link_pure_python_package "$REPO_ROOT/extensions/rcs_robotiq2f85/src/rcs_robotiq2f85" "$SITE_PACKAGES/rcs_robotiq2f85"
link_pure_python_package "$REPO_ROOT/extensions/rcs_zed/src/rcs_zed" "$SITE_PACKAGES/rcs_zed"
