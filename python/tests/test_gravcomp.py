import pytest
from rcs.envs.configs import EmptyWorldDroid
from rcs.sim.composer import ModelComposer


def _compensated_bodies(gravcomp_ignore: set[str]) -> set[str]:
    scene = EmptyWorldDroid()
    cfg = scene.config()
    cfg.add_gravcomp = True
    cfg.gravcomp_ignore = gravcomp_ignore
    composer = scene.create_model(cfg)
    assert isinstance(composer, ModelComposer)
    model = composer.get_model()
    return {model.body(i).name for i in range(model.nbody) if model.body_gravcomp[i] > 0}


def test_gravcomp_ignore_excludes_only_the_given_attachments():
    compensated = _compensated_bodies(set())
    # the wrist camera (camera_adds) and its mount (robot_frame_objects) are compensated by default
    assert {"wrist_zed_mini", "zed_mount_base"} <= compensated

    ignored = _compensated_bodies({"wrist", "zed_mount"})
    assert ignored == compensated - {"wrist_zed_mini", "zed_mount_base"}
    # the arm and the gripper stay compensated
    assert any(name.startswith("rightfr3_link") for name in ignored)
    assert any(name.startswith("gripper") for name in ignored)


def test_gravcomp_ignore_rejects_unknown_ids():
    with pytest.raises(ValueError, match="missing"):
        _compensated_bodies({"missing"})
