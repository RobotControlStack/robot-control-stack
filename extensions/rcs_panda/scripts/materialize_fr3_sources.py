from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover
    import tomli as tomllib  # type: ignore


REPO_URL = "https://github.com/RobotControlStack/robot-control-stack.git"


def load_version(project_root: Path) -> str:
    data = tomllib.loads((project_root / "pyproject.toml").read_text())
    return data["project"]["version"]


def find_shared_source(project_root: Path) -> Path:
    env_override = os.environ.get("RCS_FR3_SOURCE_DIR")
    if env_override:
        candidate = Path(env_override).resolve()
        if (candidate / "CMakeLists.txt").is_file():
            return candidate
        msg = f"RCS_FR3_SOURCE_DIR does not point to a valid FR3 source tree: {candidate}"
        raise FileNotFoundError(msg)

    sibling = (project_root.parent / "rcs_fr3" / "src").resolve()
    if (sibling / "CMakeLists.txt").is_file():
        return sibling

    version = load_version(project_root)
    tmpdir = Path(tempfile.mkdtemp(prefix="rcs_panda_fr3_"))
    try:
        subprocess.run(
            ["git", "clone", "--depth", "1", "--branch", f"v{version}", REPO_URL, str(tmpdir)],
            check=True,
        )
    except subprocess.CalledProcessError:
        shutil.rmtree(tmpdir, ignore_errors=True)
        tmpdir = Path(tempfile.mkdtemp(prefix="rcs_panda_fr3_"))
        subprocess.run(
            ["git", "clone", "--depth", "1", REPO_URL, str(tmpdir)],
            check=True,
        )
    cloned = tmpdir / "extensions" / "rcs_fr3" / "src"
    if not (cloned / "CMakeLists.txt").is_file():
        msg = f"Could not locate FR3 shared sources in cloned repo at {cloned}"
        raise FileNotFoundError(msg)
    return cloned


def copy_file(src: Path, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)


def materialize(project_root: Path, output_dir: Path) -> None:
    shared_src = find_shared_source(project_root)
    panda_src = project_root / "src_fr3"

    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    copy_file(shared_src / "CMakeLists.txt", output_dir / "CMakeLists.txt")
    shutil.copytree(shared_src / "hw", output_dir / "hw")
    copy_file(panda_src / "pybind" / "CMakeLists.txt", output_dir / "pybind" / "CMakeLists.txt")
    copy_file(shared_src / "pybind" / "rcs.cpp", output_dir / "pybind" / "rcs.cpp")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    materialize(args.project_root.resolve(), args.output_dir.resolve())


if __name__ == "__main__":
    main()
