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
    ContourParallelOptions,
    DirectParallelOptions,
    NonEquidistantOptions,
    # Solutions
    SharpTurnSolution,
    UnderFillSolution,
    # Main classes
    Curve,
    FileAgent,
    NEPathPlanner,
)

# Import demo functions
from . import demos

__all__ = [
    "_nepath",
    "ConnectAlgorithm",
    "ContourParallelOptions",
    "Curve",
    "DirectParallelOptions",
    "FileAgent",
    "NEPathPlanner",
    "NonEquidistantOptions",
    "Path",
    "SharpTurnSolution",
    "UnderFillSolution",
    "demos",
]
