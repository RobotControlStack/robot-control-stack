#!/bin/sh
set -eu

REPO_ROOT="${1:-/workspace/robot-control-stack}"

if [ ! -d "$REPO_ROOT" ]; then
    echo "Mounted repo not found at $REPO_ROOT; leaving installed packages unchanged."
    exit 0
fi

SITE_PACKAGES="$(python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"

link_compiled_package() {
    src_dir="$1"
    dst_dir="$2"

    if [ ! -d "$src_dir" ] || [ ! -d "$dst_dir" ]; then
        return
    fi

    tmp_keep="$(mktemp -d)"
    find "$dst_dir" -maxdepth 1 \( -name '_core*.so' -o -name 'lib*.so*' \) -exec mv {} "$tmp_keep/" \;
    rm -rf "$dst_dir"
    mkdir -p "$dst_dir"
    cp -as "$src_dir/." "$dst_dir/"
    find "$tmp_keep" -maxdepth 1 -type f -exec mv {} "$dst_dir/" \;
    rmdir "$tmp_keep"
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

link_compiled_package "$REPO_ROOT/python/rcs" "$SITE_PACKAGES/rcs"
link_compiled_package "$REPO_ROOT/extensions/rcs_fr3/src/rcs_fr3" "$SITE_PACKAGES/rcs_fr3"
link_pure_python_package "$REPO_ROOT/extensions/rcs_realsense/src/rcs_realsense" "$SITE_PACKAGES/rcs_realsense"
link_pure_python_package "$REPO_ROOT/extensions/rcs_robotiq2f85/src/rcs_robotiq2f85" "$SITE_PACKAGES/rcs_robotiq2f85"
link_pure_python_package "$REPO_ROOT/extensions/rcs_zed/src/rcs_zed" "$SITE_PACKAGES/rcs_zed"
link_pure_python_package "$REPO_ROOT/vlagents" "$SITE_PACKAGES/vlagents"
