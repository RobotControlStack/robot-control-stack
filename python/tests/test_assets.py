import io
import zipfile
from pathlib import Path

import pytest

import rcs

URL = "https://example.com/{tag}.zip"


class _FakeResponse:
    def __init__(self, content: bytes):
        self.content = content

    def raise_for_status(self):
        pass


def _archive(files: dict[str, str]) -> bytes:
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w") as archive:
        for name, content in files.items():
            archive.writestr(f"robot-control-stack-1.0.0/{name}", content)
    return buffer.getvalue()


@pytest.fixture()
def downloads(monkeypatch: pytest.MonkeyPatch) -> list[str]:
    """Serves an archive with a single scene file and records the requested urls."""
    urls: list[str] = []

    def fake_get(url: str, timeout: float) -> _FakeResponse:  # noqa: ARG001
        urls.append(url)
        return _FakeResponse(_archive({"assets/scene.xml": "<mujoco/>", "README.md": ""}))

    monkeypatch.setattr(rcs.requests, "get", fake_get)
    return urls


def _get_prefix(env_prefix: Path | None, tmp_path: Path, version: str = "1.0.0") -> str:
    return rcs.get_prefix(str(env_prefix) if env_prefix else None, tmp_path / "pkg", tmp_path / "default", version, URL)


def test_assets_downloaded_once(tmp_path: Path, downloads: list[str]):
    for _ in range(3):
        assert _get_prefix(None, tmp_path) == str(tmp_path / "default")
    assert downloads == ["https://example.com/v1.0.0.zip"]
    assets_dir = tmp_path / "default" / "assets"
    assert (assets_dir / "scene.xml").read_text() == "<mujoco/>"
    assert (assets_dir / rcs.ASSETS_VERSION_FILE).read_text() == "1.0.0"
    assert not (tmp_path / "default" / "README.md").exists()
    # the staging directory is cleaned up
    assert [p.name for p in (tmp_path / "default").iterdir()] == ["assets"]


def test_assets_replaced_on_version_change(tmp_path: Path, downloads: list[str]):
    _get_prefix(None, tmp_path, version="1.0.0")
    stale_file = tmp_path / "default" / "assets" / "removed_in_new_version.xml"
    stale_file.write_text("")
    _get_prefix(None, tmp_path, version="1.1.0")
    assert downloads == ["https://example.com/v1.0.0.zip", "https://example.com/v1.1.0.zip"]
    assert (tmp_path / "default" / "assets" / rcs.ASSETS_VERSION_FILE).read_text() == "1.1.0"
    assert not stale_file.exists()


def test_unversioned_default_assets_redownloaded(tmp_path: Path, downloads: list[str]):
    # e.g. left by an older rcs version or an interrupted download
    (tmp_path / "default" / "assets").mkdir(parents=True)
    _get_prefix(None, tmp_path)
    assert len(downloads) == 1
    assert (tmp_path / "default" / "assets" / rcs.ASSETS_VERSION_FILE).is_file()


def test_unversioned_env_prefix_used_as_is(tmp_path: Path, downloads: list[str]):
    # e.g. RCS_PREFIX pointing to a source checkout
    checkout = tmp_path / "checkout"
    (checkout / "assets").mkdir(parents=True)
    assert _get_prefix(checkout, tmp_path) == str(checkout)
    assert downloads == []
    assert not (checkout / "assets" / rcs.ASSETS_VERSION_FILE).exists()


def test_env_prefix_downloaded_when_missing(tmp_path: Path, downloads: list[str]):
    prefix = tmp_path / "env"
    for _ in range(2):
        assert _get_prefix(prefix, tmp_path) == str(prefix)
    assert len(downloads) == 1


def test_editable_install_skips_download(tmp_path: Path, downloads: list[str]):
    (tmp_path / "pkg" / "assets").mkdir(parents=True)
    assert _get_prefix(None, tmp_path) == str(tmp_path / "pkg")
    assert downloads == []


def test_archive_without_assets_keeps_existing_assets(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    assets_dir = tmp_path / "prefix" / "assets"
    assets_dir.mkdir(parents=True)
    (assets_dir / "scene.xml").write_text("old")
    monkeypatch.setattr(rcs.requests, "get", lambda *_, **__: _FakeResponse(_archive({"README.md": ""})))
    with pytest.raises(FileNotFoundError):
        rcs.download_assets("1.0.0", tmp_path / "prefix", URL)
    assert (assets_dir / "scene.xml").read_text() == "old"
    assert [p.name for p in (tmp_path / "prefix").iterdir()] == ["assets"]
