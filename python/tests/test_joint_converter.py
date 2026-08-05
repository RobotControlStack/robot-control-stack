import numpy as np
import pandas as pd

from rcs.lerobot_joint_converter import JointDatasetConverter


class _TestConverter(JointDatasetConverter):
    def __init__(self, *, disable_stationary_frame_filtering: bool):
        self.disable_stationary_frame_filtering = disable_stationary_frame_filtering

    def _build_observation_state(self, row: pd.Series) -> np.ndarray:
        return np.asarray([row["step"]], dtype=np.float32)

    def _convert_action_to_joint_space(self, row: pd.Series) -> np.ndarray:
        return np.asarray(row["raw_action"], dtype=np.float32)


def _transition_table() -> pd.DataFrame:
    return pd.DataFrame(
        {
            "step": [0, 1, 2],
            "raw_action": [
                [1.0, 2.0],
                [1.0, 2.0],
                [1.0, 2.001],
            ],
        }
    )


def test_prepare_transition_table_filters_stationary_frames_by_default():
    converter = _TestConverter(disable_stationary_frame_filtering=False)

    result = converter._prepare_transition_table(_transition_table())

    assert result["step"].tolist() == [0, 2]


def test_prepare_transition_table_keeps_stationary_frames_when_filtering_is_disabled():
    converter = _TestConverter(disable_stationary_frame_filtering=True)

    result = converter._prepare_transition_table(_transition_table())

    assert result["step"].tolist() == [0, 1, 2]
