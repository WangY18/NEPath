"""
NEPath Bindings
===============

Python bindings for NEPath C++ code.
"""

__version__ = "0.1.0"

# Import C++ extensions
from . import _nepath

# Import all classes and functions from _nepath
from ._nepath import (
    # Enums
    ConnectAlgorithm,

    # Data structures
    Path,

    # Options
    DirectParallelOptions,
    ContourParallelOptions,
    NonEquidistantOptions,

    # Solutions
    UnderFillSolution,
    SharpTurnSolution,

    # Main classes
    NEPathPlanner,
    FileAgent,
    Curve,
)

# Import demo functions
from . import demos

__all__ = [
    "_nepath",
    "ConnectAlgorithm",
    "Path",
    "DirectParallelOptions",
    "ContourParallelOptions",
    "NonEquidistantOptions",
    "UnderFillSolution",
    "SharpTurnSolution",
    "NEPathPlanner",
    "FileAgent",
    "Curve",
    "demos",
]
