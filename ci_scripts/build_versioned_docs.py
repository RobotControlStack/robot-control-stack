#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
import tomllib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DOCS_DIR = REPO_ROOT / "docs"
OUTPUT_DIR = DOCS_DIR / "_build" / "site"
BASE_URL = os.environ.get("RCS_DOCS_BASE_URL", "https://robotcontrolstack.org").rstrip("/")
TAG_FILTER = [tag.strip() for tag in os.environ.get("RCS_DOCS_TAGS", "").split(",") if tag.strip()]


def run(*args: str, cwd: Path = REPO_ROOT, env: dict[str, str] | None = None) -> str:
    result = subprocess.run(args, cwd=cwd, env=env, check=True, text=True, capture_output=True)
    return result.stdout.strip()


def list_release_tags() -> list[str]:
    if TAG_FILTER:
        return TAG_FILTER
    tags = run("git", "tag", "--list", "v*", "--sort=-version:refname")
    return [tag for tag in tags.splitlines() if tag]


def has_docs(ref: str) -> bool:
    try:
        run("git", "cat-file", "-e", f"{ref}:docs/conf.py")
        return True
    except subprocess.CalledProcessError:
        return False


def read_release(repo_root: Path) -> str:
    with (repo_root / "pyproject.toml").open("rb") as f:
        return tomllib.load(f)["project"]["version"]


def build_docs(repo_root: Path, output_dir: Path, version_match: str) -> None:
    env = os.environ.copy()
    env["RCS_DOCS_VERSION"] = version_match
    env["RCS_DOCS_RELEASE"] = read_release(repo_root)
    subprocess.run(
        ["sphinx-build", "-b", "html", "docs", str(output_dir)],
        cwd=repo_root,
        env=env,
        check=True,
        text=True,
    )


def sync_release_build_config(repo_root: Path) -> None:
    shutil.copy2(REPO_ROOT / "docs" / "conf.py", repo_root / "docs" / "conf.py")

    static_dir = repo_root / "docs" / "_static"
    static_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(REPO_ROOT / "docs" / "_static" / "version_switcher.json", static_dir / "version_switcher.json")


def overwrite_switcher_json(site_dir: Path, entries: list[dict[str, str]]) -> None:
    payload = json.dumps(entries, indent=4) + "\n"
    for root in [site_dir, site_dir / "latest", *[p for p in site_dir.iterdir() if p.is_dir() and p.name not in {"latest", "_sources", "_static"}]]:
        static_dir = root / "_static"
        if static_dir.exists():
            (static_dir / "version_switcher.json").write_text(payload)


def main() -> None:
    if OUTPUT_DIR.exists():
        shutil.rmtree(OUTPUT_DIR)
    OUTPUT_DIR.mkdir(parents=True)

    entries: list[dict[str, str]] = [
        {
            "name": "latest",
            "version": "latest",
            "url": f"{BASE_URL}/",
        }
    ]

    with tempfile.TemporaryDirectory(prefix="rcs-docs-versioned-") as temp_dir_str:
        temp_dir = Path(temp_dir_str)

        latest_dir = OUTPUT_DIR / "latest"
        build_docs(REPO_ROOT, latest_dir, "latest")

        for tag in list_release_tags():
            if not has_docs(tag):
                continue

            worktree_dir = temp_dir / tag
            subprocess.run(["git", "worktree", "add", "--detach", str(worktree_dir), tag], cwd=REPO_ROOT, check=True)
            try:
                sync_release_build_config(worktree_dir)
                release = read_release(worktree_dir)
                release_dir = OUTPUT_DIR / release
                build_docs(worktree_dir, release_dir, release)
                entries.append(
                    {
                        "name": release,
                        "version": release,
                        "url": f"{BASE_URL}/{release}/",
                    }
                )
            finally:
                subprocess.run(["git", "worktree", "remove", "--force", str(worktree_dir)], cwd=REPO_ROOT, check=True)

    shutil.copytree(latest_dir, OUTPUT_DIR, dirs_exist_ok=True)
    overwrite_switcher_json(OUTPUT_DIR, entries)


if __name__ == "__main__":
    main()
