"""
Type stubs for NEPath C++ bindings.

This file provides type information for the nanobind-wrapped C++ extension.
"""

from __future__ import annotations

from enum import Enum
from typing import overload

import numpy as np
from numpy.typing import NDArray


class ConnectAlgorithm(Enum):
    """Connection algorithm for toolpath generation."""
    none = 0
    cfs = 1  # Connected Fermat Spiral
    dfs = 2  # Depth First Search


class Path:
    """A 2D toolpath represented as arrays of x and y coordinates."""
    
    length: int
    """Number of waypoints in the path."""
    
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, other: Path) -> None: ...
    
    def xmax(self) -> float:
        """Find the maximum x-coordinate."""
        ...
    
    def xmin(self) -> float:
        """Find the minimum x-coordinate."""
        ...
    
    def ymax(self) -> float:
        """Find the maximum y-coordinate."""
        ...
    
    def ymin(self) -> float:
        """Find the minimum y-coordinate."""
        ...
    
    def get_arrays(self) -> tuple[list[float], list[float]]:
        """Get x and y coordinates as Python lists."""
        ...
    
    def set_arrays(
        self,
        x: NDArray[np.float64],
        y: NDArray[np.float64],
    ) -> None:
        """Set x and y coordinates from numpy arrays."""
        ...
    
    @staticmethod
    def from_arrays(
        x: NDArray[np.float64],
        y: NDArray[np.float64],
    ) -> Path:
        """Create a Path from numpy arrays."""
        ...


class DirectParallelOptions:
    """Options for direction-parallel (Raster/Zigzag) toolpath generation."""
    
    delta: float
    """Line width."""
    
    angle: float
    """Angle (rad) between toolpaths and x-axis."""
    
    def __init__(self) -> None: ...


class ContourParallelOptions:
    """Options for contour-parallel (CP) toolpath generation."""
    
    delta: float
    """Line width."""
    
    wash: bool
    """Enable resampling."""
    
    washdis: float
    """Resampling distance."""
    
    num_least: int
    """Minimum number of waypoints."""
    
    connect: ConnectAlgorithm
    """Connection algorithm."""
    
    def __init__(self) -> None: ...


class NonEquidistantOptions:
    """Options for non-equidistant (IQOP) toolpath generation."""
    
    delta: float
    """Upper bound of delta_i."""
    
    alpha: float
    """Lower bound of delta_i."""
    
    dot_delta: float
    """Upper bound of dot(delta_i)."""
    
    ddot_delta: float
    """Upper bound of ddot(delta_i)."""
    
    optimize_Q: bool
    """Optimize isoperimetric quotient."""
    
    optimize_S: bool
    """Optimize area."""
    
    optimize_L: bool
    """Optimize length."""
    
    lambda_Q: float
    """Weighting coefficient for Q."""
    
    lambda_S: float
    """Weighting coefficient for S."""
    
    lambda_L: float
    """Weighting coefficient for L."""
    
    epsilon: float
    """Maximum error of offsetting distances."""
    
    step_max: int
    """Maximum iteration steps."""
    
    wash: bool
    """Enable resampling."""
    
    washdis: float
    """Resampling distance."""
    
    num_least: int
    """Minimum number of waypoints."""
    
    connect: ConnectAlgorithm
    """Connection algorithm."""
    
    def __init__(self) -> None: ...


class UnderFillSolution:
    """Solution containing underfill calculation results."""
    
    nx: int
    """Length of xs."""
    
    ny: int
    """Length of ys."""
    
    underfillrate: float
    """Underfill rate (0.0 to 1.0)."""
    
    def __init__(self) -> None: ...
    
    def get_xs(self) -> list[float]:
        """Get xs array."""
        ...
    
    def get_ys(self) -> list[float]:
        """Get ys array."""
        ...
    
    def get_map_slice(self) -> list[list[bool]]:
        """Get map_slice as 2D list."""
        ...
    
    def get_map_delta(self) -> list[list[bool]]:
        """Get map_delta as 2D list."""
        ...
    
    def clear(self) -> None:
        """Clear the solution."""
        ...


class SharpTurnSolution:
    """Solution containing sharp turn detection results."""
    
    length: int
    """Number of waypoints."""
    
    radius: float
    """Radius of the circle."""
    
    threshold: float
    """Threshold of area to determine sharp corners."""
    
    close: bool
    """Whether the toolpath is closed."""
    
    def __init__(self) -> None: ...
    
    def get_area_percent(self) -> list[float]:
        """Get AreaPercent array."""
        ...
    
    def get_sharp_turn(self) -> list[bool]:
        """Get SharpTurn array."""
        ...
    
    def clear(self) -> None:
        """Clear the solution."""
        ...


class NEPathPlanner:
    """Main class for toolpath planning."""
    
    contour: Path
    """The contour of the slice."""
    
    holes: list[Path]
    """The holes of the slice."""
    
    def __init__(self) -> None: ...
    
    @overload
    def set_contour(
        self,
        contour: Path,
        wash: bool = True,
        washdis: float = 0.2,
        num_least: int = 50,
    ) -> None:
        """Set the contour (outer boundary) of the slice."""
        ...
    
    @overload
    def set_contour(
        self,
        x: NDArray[np.float64],
        y: NDArray[np.float64],
        wash: bool = True,
        washdis: float = 0.2,
        num_least: int = 50,
    ) -> None:
        """Set the contour from numpy arrays."""
        ...
    
    @overload
    def addhole(
        self,
        hole: Path,
        wash: bool = True,
        washdis: float = 0.2,
        num_least: int = 50,
    ) -> None:
        """Add a new hole (inner boundary) onto the slice."""
        ...
    
    @overload
    def addhole(
        self,
        x: NDArray[np.float64],
        y: NDArray[np.float64],
        wash: bool = True,
        washdis: float = 0.2,
        num_least: int = 50,
    ) -> None:
        """Add a hole from numpy arrays."""
        ...
    
    def addholes(
        self,
        holes: list[Path],
        wash: bool = True,
        washdis: float = 0.2,
        num_least: int = 50,
    ) -> None:
        """Add multiple holes (inner boundaries) onto the slice."""
        ...
    
    def tool_compensate(self, opts: ContourParallelOptions) -> list[Path]:
        """Offset the contour and holes of the slice with a distance."""
        ...
    
    def Raster(self, opts: DirectParallelOptions) -> list[Path]:
        """Generate Raster toolpath."""
        ...
    
    def Zigzag(self, opts: DirectParallelOptions) -> list[Path]:
        """Generate Zigzag toolpath."""
        ...
    
    def CP(self, opts: ContourParallelOptions) -> list[Path]:
        """Generate CP (Contour Parallel) toolpath."""
        ...
    
    def IQOP(self, opts: NonEquidistantOptions, log: bool = True) -> list[Path]:
        """Generate IQOP (Isoperimetric Quotient Optimization) toolpath."""
        ...


class FileAgent:
    """File I/O utilities for paths."""
    
    @staticmethod
    def read_csv(filename: str) -> Path:
        """Read a path from a CSV file."""
        ...
    
    @overload
    @staticmethod
    def write_csv(path: Path, filename: str) -> None:
        """Write a path to a CSV file."""
        ...
    
    @overload
    @staticmethod
    def write_csv(
        paths: list[Path],
        filename_pre: str,
        filename_post: str | None = None,
    ) -> None:
        """Write paths to CSV files with prefix and suffix."""
        ...
    
    @staticmethod
    def delete_AllFiles(path: str) -> None:
        """Delete all files in the folder."""
        ...
    
    @staticmethod
    def mkdir(path: str, clear: bool = False) -> None:
        """Create a directory."""
        ...


class Curve:
    """Curve analysis utilities."""
    
    @staticmethod
    def UnderFill(
        contour: Path,
        holes: list[Path],
        ps: list[Path],
        delta: float,
        reratio: float,
    ) -> UnderFillSolution:
        """Calculate the underfill."""
        ...
    
    @staticmethod
    def UnderFillRate(
        contour: Path,
        holes: list[Path],
        ps: list[Path],
        delta: float,
        reratio: float,
    ) -> float:
        """Calculate the underfill rate."""
        ...
    
    @staticmethod
    def SharpTurn_Invariant(
        p: Path,
        radius: float,
        threshold: float = 0.3,
        close: bool = False,
        washdis: float = -1.0,
    ) -> SharpTurnSolution:
        """Calculate the sharp corners."""
        ...
    
    @staticmethod
    def SharpTurnNum_Invariant(
        p: Path,
        radius: float,
        threshold: float = 0.3,
        close: bool = False,
        washdis: float = -1.0,
    ) -> int:
        """Calculate the number of sharp corners."""
        ...
    
    @staticmethod
    def wash_dis(p: Path, dis: float) -> Path:
        """Resample the path with a distance approximately equal to dis."""
        ...
    
    @staticmethod
    def AreaCal(x: list[float], y: list[float], length: int) -> float:
        """Calculate area enclosed by path."""
        ...
    
    @staticmethod
    def TotalLength(
        x: list[float],
        y: list[float],
        length: int,
        poly: bool = True,
    ) -> float:
        """Calculate length of path."""
        ...
