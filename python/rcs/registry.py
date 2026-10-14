"""Factory registries for pluggable hardware: cameras, grippers and hands.

An extension that provides a backend declares a factory as an entry point in its own
``pyproject.toml`` and nothing else has to know about it::

    [project.entry-points."rcs.cameras"]
    realsense = "rcs_realsense.creators:create_camera_set"

The entry point name is the type id that configs already carry (``camera_type_id``,
``GripperType.id``, ``HandType.id``). Entry points are plain package metadata, so listing the
available backends imports nothing, and a backend is imported only when its id is requested. A
backend that is not installed is simply absent from the metadata, which is how a missing
extension surfaces: as an unknown id, with the installed ids listed next to it.

Two installed distributions may provide the same id, for example `rcs_fr3` and `rcs_panda` both
implement `FrankaHand` against different libfranka versions. Resolving that silently would pick an
arbitrary one, so it is an error instead; `register` settles it explicitly for the process.
"""

from collections.abc import Callable
from importlib.metadata import entry_points
from typing import TYPE_CHECKING, Generic, TypeVar

if TYPE_CHECKING:
    from rcs._core.common import Gripper, Hand
    from rcs.camera.hw import HardwareCamera

T = TypeVar("T")


class Registry(Generic[T]):
    """Maps type ids to factories, filled from an entry point group and by `register`."""

    def __init__(self, group: str) -> None:
        self._group = group
        self._factories: dict[str, Callable[..., T]] = {}

    def register(self, type_id: str, factory: Callable[..., T]) -> None:
        """Add a factory at runtime, for tests and for backends not installed as a distribution."""
        self._factories[type_id] = factory

    def available(self) -> set[str]:
        """Ids that can be resolved, from registered factories and installed entry points. Imports nothing."""
        return set(self._factories) | {ep.name for ep in entry_points(group=self._group)}

    def get(self, type_id: str) -> Callable[..., T]:
        """Return the factory for `type_id`, importing its backend on first use."""
        if type_id not in self._factories:
            matches = list(entry_points(group=self._group, name=type_id))
            if len(matches) > 1:
                dists = sorted({ep.dist.name for ep in matches if ep.dist is not None})
                msg = (
                    f"{self._group} type {type_id!r} is provided by more than one installed package {dists}, "
                    f"call `register({type_id!r}, factory)` to choose one"
                )
                raise ValueError(msg)
            if matches:
                self._factories[type_id] = matches[0].load()
        try:
            return self._factories[type_id]
        except KeyError:
            msg = f"Unknown {self._group} type {type_id!r}, available: {sorted(self.available())}"
            raise ValueError(msg) from None


CAMERAS: "Registry[HardwareCamera]" = Registry("rcs.cameras")
GRIPPERS: "Registry[Gripper]" = Registry("rcs.grippers")
HANDS: "Registry[Hand]" = Registry("rcs.hands")

__all__ = ["CAMERAS", "GRIPPERS", "HANDS", "Registry"]
