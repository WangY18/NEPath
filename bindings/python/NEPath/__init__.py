# NEPath/__init__.py

"""
NEPath Python package.

Expose C++ NEPath library functionalities via pybind11.
"""

# Import core extension module
from .NEPathConfig import IncludeIpopt, IncludeGurobi


if IncludeGurobi:
    import platform

    if platform.system() == "Windows":
        import os

        gurobi_home = os.environ.get("GUROBI_HOME")
        if gurobi_home:
            gurobi_dll_path = os.path.join(gurobi_home, "bin")
            if os.path.isdir(gurobi_dll_path):
                os.add_dll_directory(gurobi_dll_path)

from ._nepath import *

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
