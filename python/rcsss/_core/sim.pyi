# ATTENTION: auto generated from C++ code, use `make stubgen` to update!
"""
sim module
"""
from __future__ import annotations

import typing

import rcsss._core.common

__all__ = ["FR3", "FR3Config", "FR3State"]

class FR3(rcsss._core.common.Robot):
    def __init__(self, mjmdl: str, rlmdl: str, render: bool | None) -> None: ...
    def reset(self) -> None: ...

class FR3Config(rcsss._core.common.RConfig):
    ik_duration: int
    def __init__(self) -> None: ...

class FR3State(rcsss._core.common.RState):
    def __init__(self) -> None: ...
    @property
    def collision(self) -> bool: ...
    @property
    def ik_success(self) -> bool: ...
