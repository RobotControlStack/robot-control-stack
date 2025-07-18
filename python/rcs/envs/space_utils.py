from typing import (
    Annotated,
    Any,
    Literal,
    SupportsFloat,
    Type,
    TypeAlias,
    TypedDict,
    TypeVar,
    get_args,
    get_origin,
    get_type_hints,
)

import gymnasium as gym
import numpy as np

M = TypeVar("M", bound=int)
VecType: TypeAlias = np.ndarray[M, np.dtype[np.float64]]
Vec7Type: TypeAlias = np.ndarray[tuple[Literal[7]], np.dtype[np.float64]]
Vec3Type: TypeAlias = np.ndarray[tuple[Literal[3]], np.dtype[np.float64]]
Vec6Type: TypeAlias = np.ndarray[tuple[Literal[6]], np.dtype[np.float64]]
Vec18Type: TypeAlias = np.ndarray[tuple[Literal[18]], np.dtype[np.float64]]


class RCSpaceType(TypedDict): ...


def get_space(
    tp: Type[RCSpaceType],
    params: dict[str, dict[str, Any]] | None = None,
    child_dict_keys_to_unfold: dict[str, list[str]] | None = None,
) -> gym.spaces.Dict:
    """Generates Gym Space from given annotated type.

    Args:
        tp (RCSpaceType): Space type as TypedDict (should inherit from RCSpaceType) with gym spaces annotated.
            See the examples below as reference.
        params (dict[str, dict[str, Any]] | None, optional): Parameters with which the spaces should be populated if they are given as lambdas.
            If used, the second annotation argument must be a string matching the key of the first dict. The keys of the first dict can also
            be a path. See the examples below as reference. Defaults to None.
        child_dict_keys_to_unfold (dict[str, list[str]] | None, optional): Keys for Gym Dict Spaces that should be unfolded.
            See the examples below for reference. Defaults to None.

    Returns:
        gym.spaces.Dict: The calculated gym space with unfolded dictionaries.


    Lets create a simple example space type which has the space annotated to the type:
    ```python
    class SimpleSpace(RCSpaceType):
        my_int: Annotated[
            int,
            gym.spaces.Discrete(1),
        ]
        my_float: Annotated[
            float,
            gym.spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32),
        ]
    ```
    >>> get_space(SimpleSpace)
    Dict('my_float': Box(0.0, 1.0, (1,), float32), 'my_int': Discrete(1))

    We can also create a parameterize space by using a lambda function as the annotation
    combined with an identifier string as the second argument. This can for example be useful
    when you have a camera but don't know its resolution beforehand.
    ```python
    class SimpleSpaceWithLambda(RCSpaceType):
        image: Annotated[
            np.ndarray,
            lambda height, width: gym.spaces.Box(low=0, high=255, shape=(height, width, 3), dtype=np.uint8),
            "image"
        ]
    ```
    We can then specify the parameters with the `params` argument delivered
    with a dict that contains the kwargs for the lambda function:

    >>> get_space(SimpleSpaceWithLambda, params={"image": {"height": 480, "width": 640}})
    Dict('image': Box(0, 255, (480, 640, 3), uint8))

    We can also create nested spaces with dictionaries that should be unfolded. The key has
    type has to be annotated with an identifier string. For example when you have
    several robots, of which we want to specify their joint position under the key "robots_joints":
    ```python
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
    ```
    The provide `child_dict_keys_to_unfold` argument which maps the identifier string to the keys that should be unfolded.
    >>> get_space(SimpleNestedSpace, child_dict_keys_to_unfold={"robots": ["robot1", "robot2"]})
    Dict('robots_joints': Dict('robot1': Box(-3.1415927, 3.1415927, (7,), float32), 'robot2': Box(-3.1415927, 3.1415927, (7,), float32)))


    This advanced example defines a nested dict space.
    ```python
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
    ```
    The nested dict can be unfolded either symmetrically by providing the key names for the identifiers as in the example above, or
    like below for each tree branch differently by providing the path (given by previous unfolded keys) followed by the identifier for the node.
    >>> get_space(
    >>>     AdvancedNestedSpace,
    >>>     child_dict_keys_to_unfold={
    >>>         "cams": ["cam1", "cam2"],
    >>>         "/cam1/cam_type": ["depth", "rgb"],
    >>>         "/cam2/cam_type": ["rgb"],
    >>>     },
    >>> )
    Dict('frames': Dict('cam1': Dict('depth': Box(0, 255, (480, 640, 3), uint8), 'rgb': Box(0, 255, (480, 640, 3), uint8)), 'cam2': Dict('rgb': Box(0, 255, (480, 640, 3), uint8))))

    The same path logic can be used for parameterized space types as well:
    ```python
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
    ```
    >>> get_space(
    >>>     AdvancedNestedSpaceWithLambda,
    >>>     child_dict_keys_to_unfold={
    >>>         "cams": ["cam1", "cam2"],
    >>>         "/cam1/cam_type": ["depth", "rgb"],
    >>>         "/cam2/cam_type": ["rgb"],
    >>>     },
    >>>     params={"/cam1/rgb/frames": {"height": 128, "width": 128}, "frames": {"height": 512, "width": 512}},
    >>> )
    Dict('frames': Dict('cam1': Dict('depth': Box(0, 255, (512, 512, 3), uint8), 'rgb': Box(0, 255, (128, 128, 3), uint8)), 'cam2': Dict('rgb': Box(0, 255, (512, 512, 3), uint8))))


    Theses classes can also be composed through inheritance:

    class Composed(AdvancedNestedSpaceWithLambda, SimpleSpace):
        ...

    >>> get_space(
    >>>     Composed,
    >>>     child_dict_keys_to_unfold={
    >>>         "cams": ["cam1", "cam2"],
    >>>         "/cam1/cam_type": ["depth", "rgb"],
    >>>         "/cam2/cam_type": ["rgb"],
    >>>     },
    >>>     params={"/cam1/rgb/frames": {"height": 128, "width": 128}, "frames": {"height": 512, "width": 512}},
    >>> )
    Dict('frames': Dict('cam1': Dict('depth': Box(0, 255, (512, 512, 3), uint8), 'rgb': Box(0, 255, (128, 128, 3), uint8)), 'cam2': Dict('rgb': Box(0, 255, (512, 512, 3), uint8))), 'my_float': Box(0.0, 1.0, (1,), float32), 'my_int': Discrete(1))

    """
    assert tp.__class__.__name__ == "_TypedDictMeta", "Type must be a TypedDict type. Hint: inherit from RCSpaceType."

    def value(t, path=""):
        if get_origin(t) == dict:
            # recursive case: space has dict subspace which keys must be populated
            assert child_dict_keys_to_unfold is not None, "No child dict keys given."
            assert len(get_args(t)) == 2, "Dict type must have two args."
            assert get_origin(get_args(t)[0]) == Annotated, "Dict key must be Annotated."
            assert get_args(get_args(t)[0])[0] == str, "Dict key must be a string."
            assert len(get_args(t)[0].__metadata__) > 0, "type annotation must have key for child_dict_keys."
            node = get_args(t)[0].__metadata__[0]
            curr_path = f"{path}/{node}"
            if curr_path in child_dict_keys_to_unfold:
                unfold_key = curr_path
            elif node in child_dict_keys_to_unfold:
                unfold_key = node
            else:
                msg = f"No matching key for child dict keys: {path}"
                raise ValueError(msg)

            return gym.spaces.Dict(
                {key: value(get_args(t)[1], f"{path}/{key}") for key in child_dict_keys_to_unfold[unfold_key]}
            )

        if not hasattr(t, "__metadata__"):
            return gym.spaces.Dict(
                {name: value(sub_t, path) for name, sub_t in get_type_hints(t, include_extras=True).items()}
            )

        if len(t.__metadata__) == 2 and callable(t.__metadata__[0]):
            # space can be parametrized and is a function
            assert params is not None, "No params given."

            node = t.__metadata__[1]
            curr_path = f"{path}/{node}"
            if curr_path in params:
                param_key = curr_path
            elif node in params:
                param_key = node
            else:
                msg = f"No matching key for child dict keys: {path}"
                raise ValueError(msg)
            space = t.__metadata__[0](**params[param_key])
            assert isinstance(space, gym.spaces.Space), "Not a gym space."
            return space
        assert isinstance(t.__metadata__[0], gym.spaces.Space), "Leaves must be gym spaces."
        return t.__metadata__[0]

    return gym.spaces.Dict({name: value(t) for name, t in get_type_hints(tp, include_extras=True).items()})


def get_space_keys(tp: Type[RCSpaceType]) -> list[str]:
    assert tp.__class__.__name__ == "_TypedDictMeta", "Type must be a TypedDict type. Hint: inherit from RCSpaceType."
    return list(get_type_hints(tp).keys())


class ActObsInfoWrapper(gym.Wrapper[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any]]):
    """Improved version of the ObservationWrapper from gymnasium. It also adds the info dict to the observation method.

    Superclass of wrappers that can modify observations using :meth:`observation` for :meth:`reset` and :meth:`step`.

    If you would like to apply a function to only the observation before
    passing it to the learning code, you can simply inherit from :class:`ObservationWrapper` and overwrite the method
    :meth:`observation` to implement that transformation. The transformation defined in that method must be
    reflected by the :attr:`env` observation space. Otherwise, you need to specify the new observation space of the
    wrapper by setting :attr:`self.observation_space` in the :meth:`__init__` method of your wrapper.

    Among others, Gymnasium provides the observation wrapper :class:`TimeAwareObservation`, which adds information about the
    index of the timestep to the observation.
    """

    def __init__(self, env: gym.Env[dict[str, Any], dict[str, Any]]):
        """Constructor for the observation wrapper."""
        gym.Wrapper.__init__(self, env)

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        """Modifies the :attr:`env` after calling :meth:`reset`, returning a modified observation using :meth:`self.observation`."""
        observation, info = self.env.reset(seed=seed, options=options)
        wrapped_obs, wrapped_info = self.observation(observation, info)
        return wrapped_obs, wrapped_info

    def step(self, action: dict[str, Any]) -> tuple[dict[str, Any], SupportsFloat, bool, bool, dict[str, Any]]:
        """Modifies the :attr:`env` after calling :meth:`step` using :meth:`self.observation` on the returned observations."""
        observation, reward, terminated, truncated, info = self.env.step(self.action(action))
        wrapped_obs, wrapped_info = self.observation(observation, info)
        return wrapped_obs, reward, terminated, truncated, wrapped_info

    def observation(self, observation: dict[str, Any], info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        """Returns a modified observation.

        Args:
            observation: The :attr:`env` observation

        Returns:
            The modified observation
        """
        return observation, info  # type: ignore

    def action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Returns a modified action before :meth:`env.step` is called.

        Args:
            action: The original :meth:`step` actions

        Returns:
            The modified actions
        """
        return action  # type: ignore
