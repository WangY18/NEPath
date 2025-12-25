# NEPath/__init__.py

"""
NEPath Python package.

Expose C++ NEPath library functionalities via pybind11.
"""

# Import core extension module
from ._nepath import *
from .NEPathConfig import IncludeIpopt, IncludeGurobi

# Optionally list public API symbols here
__all__ = [
    "ConnectAlgorithm",
    "OptimizationAlgorithm",
    "DirectParallelOptions",
    "ContourParallelOptions",
    "NonEquidistantOptions",
    "SharpTurnSolution",
    "UnderFillSolution",
    "Path",
    "Curve",
    "NEPathPlanner",
    "IncludeIpopt",
    "IncludeGurobi",
]
