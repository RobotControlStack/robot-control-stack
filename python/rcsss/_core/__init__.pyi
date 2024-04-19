# ATTENTION: auto generated from C++ code, use `make stubgen` to update!
"""

        Robot Control Stack Python Bindings
        -----------------------

        .. currentmodule:: _core

        .. autosummary::
           :toctree: _generate

    
"""
from __future__ import annotations

from . import common, hw, sim

__all__ = [
    "FrankaCommandException",
    "FrankaControlException",
    "FrankaException",
    "FrankaIncompatibleVersionException",
    "FrankaInvalidOperationException",
    "FrankaModelException",
    "FrankaNetworkException",
    "FrankaProtocolException",
    "FrankaRealtimeException",
    "common",
    "hw",
    "sim",
]

class FrankaCommandException(RuntimeError):
    pass

class FrankaControlException(RuntimeError):
    pass

class FrankaException(RuntimeError):
    pass

class FrankaIncompatibleVersionException(RuntimeError):
    pass

class FrankaInvalidOperationException(RuntimeError):
    pass

class FrankaModelException(RuntimeError):
    pass

class FrankaNetworkException(RuntimeError):
    pass

class FrankaProtocolException(RuntimeError):
    pass

class FrankaRealtimeException(RuntimeError):
    pass

__version__: str = "0.1.0"
