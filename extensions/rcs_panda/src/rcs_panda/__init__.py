import sys

from rcs_panda._core import hw

__all__ = [
    "hw",
]

# prevent gymnasium import during stubgen, which can cause a segfault
if "pybind11_stubgen" not in sys.modules:
    from rcs_panda import envs  # noqa: F401

    __all__.append("envs")
