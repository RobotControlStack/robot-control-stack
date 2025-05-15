from typing import Annotated

import gymnasium as gym
import numpy as np
from rcs.envs.space_utils import RCSpaceType, get_space, get_space_keys


class SimpleSpace(RCSpaceType):
    my_int: Annotated[
        int,
        gym.spaces.Discrete(1),
    ]
    my_float: Annotated[
        float,
        gym.spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32),
    ]


class SimpleSpaceWithLambda(RCSpaceType):
    image: Annotated[
        np.ndarray,
        lambda height, width: gym.spaces.Box(low=0, high=255, shape=(height, width, 3), dtype=np.uint8),
        "image",
    ]


class SimpleNestedSpace(RCSpaceType):
    robots_joints: dict[
        Annotated[str, "robots"],
        Annotated[
            np.ndarray,
            gym.spaces.Box(
                low=-np.pi,
                high=np.pi,
                shape=(7,),
                dtype=np.float32,
            ),
        ],
    ]


class AdvancedNestedSpace(RCSpaceType):
    frames: dict[
        Annotated[str, "cams"],
        dict[
            Annotated[str, "cam_type"],
            Annotated[
                np.ndarray,
                gym.spaces.Box(
                    low=0,
                    high=255,
                    shape=(480, 640, 3),
                    dtype=np.uint8,
                ),
            ],
        ],
    ]


class AdvancedNestedSpaceWithLambda(RCSpaceType):
    frames: dict[
        Annotated[str, "cams"],
        dict[
            Annotated[str, "cam_type"],
            Annotated[
                np.ndarray,
                lambda height, width: gym.spaces.Box(low=0, high=255, shape=(height, width, 3), dtype=np.uint8),
                "frames",
            ],
        ],
    ]


class Composed(AdvancedNestedSpaceWithLambda, SimpleSpace): ...


class TestGetSpace:

    def test_simple_space(self):
        assert get_space(SimpleSpace) == gym.spaces.Dict(
            {
                "my_int": gym.spaces.Discrete(1),
                "my_float": gym.spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32),
            }
        )

    def test_simple_space_with_lambda(self):
        assert get_space(SimpleSpaceWithLambda, params={"image": {"height": 480, "width": 640}}) == gym.spaces.Dict(
            {
                "image": gym.spaces.Box(low=0, high=255, shape=(480, 640, 3), dtype=np.uint8),
            }
        )

    def test_simple_nested_space(self):
        assert get_space(
            SimpleNestedSpace, child_dict_keys_to_unfold={"robots": ["robot1", "robot2"]}
        ) == gym.spaces.Dict(
            {
                "robots_joints": gym.spaces.Dict(
                    {
                        "robot1": gym.spaces.Box(low=-np.pi, high=np.pi, shape=(7,), dtype=np.float32),
                        "robot2": gym.spaces.Box(low=-np.pi, high=np.pi, shape=(7,), dtype=np.float32),
                    }
                ),
            }
        )

    def test_advanced_nested_space(self):

        assert get_space(
            AdvancedNestedSpace,
            child_dict_keys_to_unfold={
                "cams": ["cam1", "cam2"],
                "/cam1/cam_type": ["depth", "rgb"],
                "/cam2/cam_type": ["rgb"],
            },
        ) == gym.spaces.Dict(
            {
                "frames": gym.spaces.Dict(
                    {
                        "cam1": gym.spaces.Dict(
                            {
                                "depth": gym.spaces.Box(low=0, high=255, shape=(480, 640, 3), dtype=np.uint8),
                                "rgb": gym.spaces.Box(low=0, high=255, shape=(480, 640, 3), dtype=np.uint8),
                            }
                        ),
                        "cam2": gym.spaces.Dict(
                            {
                                "rgb": gym.spaces.Box(low=0, high=255, shape=(480, 640, 3), dtype=np.uint8),
                            }
                        ),
                    }
                ),
            }
        )

    def test_advanced_nested_space_with_lambda(self):

        assert get_space(
            AdvancedNestedSpaceWithLambda,
            child_dict_keys_to_unfold={
                "cams": ["cam1", "cam2"],
                "/cam1/cam_type": ["depth", "rgb"],
                "/cam2/cam_type": ["rgb"],
            },
            params={"/cam1/rgb/frames": {"height": 128, "width": 128}, "frames": {"height": 512, "width": 512}},
        ) == gym.spaces.Dict(
            {
                "frames": gym.spaces.Dict(
                    {
                        "cam1": gym.spaces.Dict(
                            {
                                "depth": gym.spaces.Box(low=0, high=255, shape=(512, 512, 3), dtype=np.uint8),
                                "rgb": gym.spaces.Box(low=0, high=255, shape=(128, 128, 3), dtype=np.uint8),
                            }
                        ),
                        "cam2": gym.spaces.Dict(
                            {
                                "rgb": gym.spaces.Box(low=0, high=255, shape=(512, 512, 3), dtype=np.uint8),
                            }
                        ),
                    }
                ),
            }
        )

    def test_composed_space(self):
        assert get_space(
            Composed,
            child_dict_keys_to_unfold={
                "cams": ["cam1", "cam2"],
                "/cam1/cam_type": ["depth", "rgb"],
                "/cam2/cam_type": ["rgb"],
            },
            params={"/cam1/rgb/frames": {"height": 128, "width": 128}, "frames": {"height": 512, "width": 512}},
        ) == gym.spaces.Dict(
            {
                "frames": gym.spaces.Dict(
                    {
                        "cam1": gym.spaces.Dict(
                            {
                                "depth": gym.spaces.Box(low=0, high=255, shape=(512, 512, 3), dtype=np.uint8),
                                "rgb": gym.spaces.Box(low=0, high=255, shape=(128, 128, 3), dtype=np.uint8),
                            }
                        ),
                        "cam2": gym.spaces.Dict(
                            {
                                "rgb": gym.spaces.Box(low=0, high=255, shape=(512, 512, 3), dtype=np.uint8),
                            }
                        ),
                    }
                ),
                "my_int": gym.spaces.Discrete(1),
                "my_float": gym.spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32),
            }
        )

    def test_get_space_keys(self):
        assert set(get_space_keys(SimpleSpace)) == {"my_int", "my_float"}
