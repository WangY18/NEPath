"""
NEPath Demo Functions

This module replicates the functionality from demos.cpp using the Python bindings.
"""

import numpy as np
from numpy.typing import NDArray

import NEPath as nepath
from NEPath import Path, SharpTurnSolution, UnderFillSolution

import pathlib
import shutil
import csv


# FileAgent: clear and create directory
def clear_mkdir(path: str, clear: bool = False) -> None:
    dir_path = pathlib.Path(path)

    if clear and dir_path.exists():
        # Delete directory and its contents
        print(f"Clearing existing directory: {dir_path}")
        shutil.rmtree(dir_path)

    # Ensure parent directory exists
    parent = dir_path.parent
    if not parent.exists():
        print(f"Creating parent directory: {parent}")
        parent.mkdir(parents=True, exist_ok=True)

    # Create target directory, no error if it already exists
    if not dir_path.exists():
        print(f"Creating directory: {dir_path}")
        dir_path.mkdir()


def is_csv(filename: str) -> bool:
    """Check if the filename ends with .csv (case-insensitive)"""
    return str(filename).lower().endswith(".csv")


def write_csv(p: nepath.Path, filename: str) -> None:
    """
    Write the x and y data from `p` into a CSV file.

    Parameters
    ----------
    p : nepath.Path
        Use p.get_arrays() to get x and y arrays.
    filename : str
        The output CSV file path.
    """
    if not is_csv(filename):
        print(f"{filename} is not csv file.")
        return

    # Ensure the parent directory exists
    file_path = pathlib.Path(filename)

    x, y = p.get_arrays()
    print(f"Writing CSV file: {file_path}")
    print(["x.len=", len(x), "y.len=", len(y)])
    print([(x[0], y[0])])

    # Write CSV file
    with open(file_path, "w", newline="") as out_file:
        writer = csv.writer(out_file)
        # Write header
        writer.writerow(["x", "y"])
        # Write each data row
        for j in range(len(x)):
            writer.writerow([x[j], y[j]])


def write_csv_batch(ps: list[nepath.Path], filename_pre: str, filename_post: str = ".csv"):
    """
    Write a list of `path` objects into numbered CSV files.

    Parameters
    ----------
    ps : list[nepath.Path]
        List of nepath.Path.
    filename_pre : str
        Prefix of the output CSV filename.
    filename_post : str, optional
        Suffix/extension for the file, e.g., ".csv". Default is ".csv".
    """
    if not ps:
        return

    if filename_post is None:
        filename_post = ".csv"

    n_files = len(ps)
    num_digits = 1
    n = n_files
    while n >= 10:
        num_digits += 1
        n //= 10

    for i, path_obj in enumerate(ps):
        # Build the filename with zero-padded index
        index_str = str(i).zfill(num_digits)
        filename = f"{filename_pre}/{index_str}{filename_post}"

        # Use the previous write_csv function to write the individual file
        write_csv(path_obj, filename)


def create_contour(num_points: int = 1000) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Create a sample contour (flower shape).

    Parameters
    ----------
    num_points : int
        Number of points in the contour

    Returns
    -------
    tuple[NDArray[np.float64], NDArray[np.float64]]
        x and y coordinate arrays
    """
    theta = np.linspace(0, 2 * np.pi, num_points)
    r = 15.0 * (1.0 + 0.15 * np.cos(10.0 * theta))
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y


def demo_Raster(
    delta: float = 1.0,
    angle: float = -np.pi / 3.0,
    output_dir: str = "./data_examples/demo_raster/",
) -> list[Path]:
    """
    Generate Raster toolpaths.

    Parameters
    ----------
    delta : float
        Line width of toolpaths
    angle : float
        Angle of raster toolpaths in radians
    output_dir : str
        Directory to save output files

    Returns
    -------
    list[Path]
        List of generated raster toolpaths
    """
    planner = nepath.NEPathPlanner()

    # Set the contour
    x, y = create_contour()
    planner.set_contour(x, y)

    # Set toolpath parameters
    opts = nepath.DirectParallelOptions()
    opts.delta = delta
    opts.angle = angle

    # Generate raster paths
    raster_paths = planner.Raster(opts)
    print(f"There are {len(raster_paths)} continuous toolpaths in total.")

    # Write output files
    clear_mkdir(output_dir, clear=True)
    write_csv_batch(raster_paths, output_dir, ".csv")

    return raster_paths


def demo_Zigzag(
    delta: float = 1.0,
    angle: float = np.pi / 3.0,
    output_dir: str = "./data_examples/demo_zigzag/",
) -> list[Path]:
    """
    Generate Zigzag toolpaths.

    Parameters
    ----------
    delta : float
        Line width of toolpaths
    angle : float
        Angle of zigzag toolpaths in radians
    output_dir : str
        Directory to save output files

    Returns
    -------
    list[Path]
        List of generated zigzag toolpaths
    """
    planner = nepath.NEPathPlanner()

    # Set the contour
    x, y = create_contour()
    planner.set_contour(x, y)

    # Set toolpath parameters
    opts = nepath.DirectParallelOptions()
    opts.delta = delta
    opts.angle = angle

    # Generate zigzag paths
    zigzag_paths = planner.Zigzag(opts)
    print(f"There are {len(zigzag_paths)} continuous toolpaths in total.")
    for i, path in enumerate(zigzag_paths):
        print(f"Toolpath {i} has {path.length} waypoints.")

    # Write output files
    clear_mkdir(output_dir, clear=True)
    write_csv_batch(zigzag_paths, output_dir, ".csv")

    # Write contour
    contour_path = nepath.Path.from_arrays(x, y)
    write_csv(contour_path, "./data_examples/contour.csv")
    return zigzag_paths


def demo_CP(
    delta: float = 1.0,
    washdis: float = 0.2,
    output_dir: str = "./data_examples/demo_CP/",
) -> list[Path]:
    """
    Generate Contour Parallel (CP) toolpaths.

    Parameters
    ----------
    delta : float
        Line width of toolpaths
    washdis : float
        Resampling distance for uniform waypoint distribution
    output_dir : str
        Directory to save output files

    Returns
    -------
    list[Path]
        List of generated CP toolpaths
    """
    planner = nepath.NEPathPlanner()

    # Create contour
    x, y = create_contour()

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = nepath.ContourParallelOptions()
    opts.delta = delta
    opts.wash = True
    opts.washdis = washdis

    # Generate CP paths
    CP_paths = planner.CP(opts)
    print(f"There are {len(CP_paths)} continuous toolpaths in total.")
    for i, path in enumerate(CP_paths):
        print(f"Toolpath {i} has {path.length} waypoints.")

    # Write output files
    clear_mkdir(output_dir, clear=True)
    write_csv_batch(CP_paths, output_dir, ".csv")

    return CP_paths


def demo_CP_CFS(
    delta: float = 1.0,
    washdis: float = 0.2,
    output_dir: str = "./data_examples/demo_CP_CFS/",
) -> list[Path]:
    """
    Generate Contour Parallel (CP) toolpaths with CFS connection.

    Parameters
    ----------
    delta : float
        Line width of toolpaths
    washdis : float
        Resampling distance for uniform waypoint distribution
    output_dir : str
        Directory to save output files

    Returns
    -------
    list[Path]
        List of generated CP toolpaths with CFS connection
    """
    planner = nepath.NEPathPlanner()

    # Create contour
    x, y = create_contour()

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = nepath.ContourParallelOptions()
    opts.delta = delta
    opts.wash = True
    opts.washdis = washdis
    opts.connector = nepath.ConnectAlgorithm.cfs  # Use CFS connection

    # Generate CP paths
    CP_paths = planner.CP(opts)
    print(f"There are {len(CP_paths)} continuous toolpaths in total.")
    for i, path in enumerate(CP_paths):
        print(f"Toolpath {i} has {path.length} waypoints.")

    # Write output files
    clear_mkdir(output_dir, clear=True)
    write_csv_batch(CP_paths, output_dir, ".csv")

    return CP_paths


def demo_CP_DFS(
    delta: float = 1.0,
    washdis: float = 0.2,
    output_dir: str = "./data_examples/demo_CP_DFS/",
) -> list[Path]:
    """
    Generate Contour Parallel (CP) toolpaths with DFS connection.

    Parameters
    ----------
    delta : float
        Line width of toolpaths
    washdis : float
        Resampling distance for uniform waypoint distribution
    output_dir : str
        Directory to save output files

    Returns
    -------
    list[Path]
        List of generated CP toolpaths with DFS connection
    """
    planner = nepath.NEPathPlanner()

    # Create contour
    x, y = create_contour()

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = nepath.ContourParallelOptions()
    opts.delta = delta
    opts.wash = True
    opts.washdis = washdis
    opts.connector = nepath.ConnectAlgorithm.dfs  # Use DFS connection

    # Generate CP paths
    CP_paths = planner.CP(opts)
    print(f"There are {len(CP_paths)} continuous toolpaths in total.")
    for i, path in enumerate(CP_paths):
        print(f"Toolpath {i} has {path.length} waypoints.")

    # Write output files
    clear_mkdir(output_dir, clear=True)
    write_csv_batch(CP_paths, output_dir, ".csv")

    return CP_paths


def demo_tool_compensate(
    delta_offset: float = -1.5,
    washdis: float = 0.2,
    output_dir: str = "./data_examples/demo_toolcompensate/",
) -> list[Path]:
    """
    Demonstrate tool compensation (offsetting).

    Parameters
    ----------
    delta_offset : float
        Offset distance (negative for inward offset)
    washdis : float
        Resampling distance for uniform waypoint distribution
    output_dir : str
        Directory to save output files

    Returns
    -------
    list[Path]
        List of tool-compensated paths
    """
    planner = nepath.NEPathPlanner()

    # Create contour
    x, y = create_contour()
    planner.set_contour(x, y)

    # Add a hole
    x_hole = np.array([-5, 5, 5, 0, -5], dtype=float)
    y_hole = np.array([-5, -5, 5, 0, 5], dtype=float)
    planner.addhole(x_hole, y_hole)

    # Tool compensate
    opts = nepath.ContourParallelOptions()
    opts.delta = delta_offset
    opts.wash = True
    opts.washdis = washdis
    ps_toolcompensate = planner.tool_compensate(opts)

    print(f"There are {len(ps_toolcompensate)} continuous toolpaths in total.")
    for i, path in enumerate(ps_toolcompensate):
        print(f"Toolpath {i} has {path.length} waypoints.")

    # Write output files
    clear_mkdir(output_dir, clear=True)
    write_csv_batch(ps_toolcompensate, output_dir, ".csv")
    write_csv(planner.holes[0], "./data_examples/hole.csv")

    return ps_toolcompensate


def demo_underfill(
    delta: float = 1.0,
    washdis: float = 0.2,
    reratio: float = 0.03,
    output_dir: str = "./data_examples/demo_underfill/",
) -> tuple[list[Path], UnderFillSolution]:
    """
    Demonstrate underfill calculation.

    Parameters
    ----------
    delta : float
        Line width of toolpaths
    washdis : float
        Resampling distance for uniform waypoint distribution
    reratio : float
        Resolution ratio for underfill computation
    output_dir : str
        Directory to save output files

    Returns
    -------
    tuple[list[Path], UnderFillSolution]
        Tuple of (CP paths, underfill solution)
    """
    planner = nepath.NEPathPlanner()

    # Create contour
    x, y = create_contour()

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = nepath.ContourParallelOptions()
    opts.delta = delta
    opts.wash = True
    opts.washdis = washdis

    # Generate CP paths
    CP_paths = planner.CP(opts)
    print(f"There are {len(CP_paths)} continuous toolpaths in total.")

    # Calculate underfill
    contour_path = nepath.Path.from_arrays(x, y)
    ufs = nepath.Curve.UnderFill(contour_path, [], CP_paths, delta, reratio)
    print(f"The underfill rate is {ufs.underfillrate * 100:.2f}%.")

    # Write output files
    clear_mkdir(output_dir / "paths/")
    write_csv_batch(CP_paths, output_dir / "paths/", ".csv")

    # Note: For underfill maps, you would need to write custom code to save xs, ys, map_slice, map_delta
    # as these require special handling in Python

    return CP_paths, ufs


def demo_sharpcorner(
    delta: float = 1.0,
    washdis: float = 0.2,
    radius: float = 1.0,
    threshold: float = 0.3,
    output_dir: str = "./data_examples/demo_sharpcorner/",
) -> tuple[list[Path], list[Path]]:
    """
    Demonstrate sharp corner detection.

    Parameters
    ----------
    delta : float
        Line width of toolpaths
    washdis : float
        Resampling distance for uniform waypoint distribution
    radius : float
        Radius of the rolling circle for sharp corner detection
    threshold : float
        Threshold of area on one side to determine a sharp corner
    output_dir : str
        Directory to save output files

    Returns
    -------
    tuple[list[Path], list[Path]]
        Tuple of (CP paths, sharp turn paths)
    """
    planner = nepath.NEPathPlanner()

    # Create contour
    x, y = create_contour()

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = nepath.ContourParallelOptions()
    opts.delta = delta
    opts.wash = True
    opts.washdis = washdis

    # Generate CP paths
    CP_paths = planner.CP(opts)
    print(f"There are {len(CP_paths)} continuous toolpaths in total.")

    # Detect sharp corners
    num = 0
    ps_sharpturn = []
    for path in CP_paths:
        sol = nepath.Curve.SharpTurn_Invariant(path, radius, threshold, True, 0.5)
        sharp_turn = sol.get_sharp_turn()
        num += sum(sharp_turn)

        # Store results (SharpTurn values as x, AreaPercent as y)
        area_percent = sol.get_area_percent()
        sharp_path = nepath.Path()
        sharp_path.set_arrays(np.array([float(st) for st in sharp_turn], dtype=float), np.array(area_percent, dtype=float))
        ps_sharpturn.append(sharp_path)

    print(f"There exist {num} sharp corners.")

    # Write output files
    clear_mkdir(output_dir / "sharpcorner/")
    write_csv_batch(ps_sharpturn, output_dir / "sharpcorner/", ".csv")
    clear_mkdir(output_dir / "paths/")
    write_csv_batch(CP_paths, output_dir / "paths/", ".csv")

    return CP_paths, ps_sharpturn


# IQOP demos (only if IPOPT is available)
try:
    # Test if IQOP is available
    test_planner = nepath.NEPathPlanner()
    if hasattr(test_planner, "IQOP"):

        def demo_IQOP_Ipopt(
            delta: float = 1.0,
            alpha: float = 0.5,
            washdis: float = 0.2,
            output_dir: str = "./data_examples/demo_IQOP_Ipopt/",
        ) -> list[Path]:
            """
            Generate IQOP toolpaths based on Ipopt.

            Parameters
            ----------
            delta : float
                Upper bound of delta_i (line width)
            alpha : float
                Scale of minimum distance
            washdis : float
                Resampling distance for uniform waypoint distribution
            output_dir : str
                Directory to save output files

            Returns
            -------
            list[Path]
                List of IQOP toolpath objects
            """
            planner = nepath.NEPathPlanner()

            # Create contour (slightly different for IQOP)
            theta = np.linspace(0, 2 * np.pi, 1000)
            r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
            x = r * np.cos(theta)
            y = r * np.sin(theta)

            # Tool compensate to get the outmost toolpath
            planner_toolcompensate = nepath.NEPathPlanner()
            planner_toolcompensate.set_contour(x, y)
            opts_toolcompensate = nepath.ContourParallelOptions()
            opts_toolcompensate.delta = -0.5 * delta
            opts_toolcompensate.wash = True
            opts_toolcompensate.washdis = washdis
            path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

            planner.set_contour(path_outmost[0])

            # Set toolpath parameters
            opts = nepath.NonEquidistantOptions()
            opts.delta = delta
            opts.alpha = alpha
            opts.dot_delta = 1.0
            opts.ddot_delta = 0.1
            opts.optimize_Q = True
            opts.optimize_S = False
            opts.optimize_L = False
            opts.lambda_Q = 1.0
            opts.wash = True
            opts.washdis = washdis
            opts.connector = nepath.ConnectAlgorithm.none  # No connection
            opts.optimizer = nepath.OptimizationAlgorithm.ipopt  # Use IPOPT solver

            # Generate IQOP paths
            IQOP_paths = planner.IQOP(opts, False)  # log=False for cleaner output
            print(f"There are {len(IQOP_paths)} continuous toolpaths in total.")

            # Write output files
            clear_mkdir(output_dir / "paths_Ipopt_IQ/")
            write_csv_batch(IQOP_paths, output_dir / "paths_Ipopt_IQ/", ".csv")
            contour_path = nepath.Path.from_arrays(x, y)
            write_csv(contour_path, output_dir / "contour.csv")

            return IQOP_paths

        def demo_IQOP_Ipopt_CFS(
            delta: float = 1.0,
            alpha: float = 0.5,
            washdis: float = 0.2,
            output_dir: str = "./data_examples/demo_IQOP_Ipopt/",
        ) -> list[Path]:
            """
            Generate IQOP toolpaths with CFS (Connected Fermat Spiral) connection based on Ipopt.

            Parameters
            ----------
            delta : float
                Upper bound of delta_i (line width)
            alpha : float
                Scale of minimum distance
            washdis : float
                Resampling distance for uniform waypoint distribution
            output_dir : str
                Directory to save output files

            Returns
            -------
            list[Path]
                List of IQOP toolpath objects with CFS connection
            """
            planner = nepath.NEPathPlanner()

            # Create contour (slightly different for IQOP)
            theta = np.linspace(0, 2 * np.pi, 1000)
            r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
            x = r * np.cos(theta)
            y = r * np.sin(theta)

            # Tool compensate to get the outmost toolpath
            planner_toolcompensate = nepath.NEPathPlanner()
            planner_toolcompensate.set_contour(x, y)
            opts_toolcompensate = nepath.ContourParallelOptions()
            opts_toolcompensate.delta = -0.5 * delta
            opts_toolcompensate.wash = True
            opts_toolcompensate.washdis = washdis
            path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

            planner.set_contour(path_outmost[0])

            # Set toolpath parameters
            opts = nepath.NonEquidistantOptions()
            opts.delta = delta
            opts.alpha = alpha
            opts.dot_delta = 1.0
            opts.ddot_delta = 0.1
            opts.optimize_Q = True
            opts.optimize_S = False
            opts.optimize_L = False
            opts.lambda_Q = 1.0
            opts.wash = True
            opts.washdis = washdis
            opts.connector = nepath.ConnectAlgorithm.none  # No connection
            opts.optimizer = nepath.OptimizationAlgorithm.ipopt  # Use IPOPT solver

            # Generate IQOP paths with CFS
            IQOP_paths = planner.IQOP(opts, False)  # log=False for cleaner output
            print(f"There are {len(IQOP_paths)} continuous toolpaths in total.")

            # Write output files
            clear_mkdir(output_dir / "path_IQOP_Ipopt_CFS/")
            write_csv_batch(IQOP_paths, output_dir / "path_IQOP_Ipopt_CFS/", ".csv")

            return IQOP_paths

        def demo_IQOP_Ipopt_DFS(
            delta: float = 1.0,
            alpha: float = 0.5,
            washdis: float = 0.2,
            output_dir: str = "./data_examples/demo_IQOP_Ipopt/",
        ) -> list[Path]:
            """
            Generate IQOP toolpaths with DFS (Depth First Search) connection based on Ipopt.

            Parameters
            ----------
            delta : float
                Upper bound of delta_i (line width)
            alpha : float
                Scale of minimum distance
            washdis : float
                Resampling distance for uniform waypoint distribution
            output_dir : str
                Directory to save output files

            Returns
            -------
            list[Path]
                List of IQOP toolpath objects with DFS connection
            """
            planner = nepath.NEPathPlanner()

            # Create contour (slightly different for IQOP)
            theta = np.linspace(0, 2 * np.pi, 1000)
            r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
            x = r * np.cos(theta)
            y = r * np.sin(theta)

            # Tool compensate to get the outmost toolpath
            planner_toolcompensate = nepath.NEPathPlanner()
            planner_toolcompensate.set_contour(x, y)
            opts_toolcompensate = nepath.ContourParallelOptions()
            opts_toolcompensate.delta = -0.5 * delta
            opts_toolcompensate.wash = True
            opts_toolcompensate.washdis = washdis
            path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

            planner.set_contour(path_outmost[0])

            # Set toolpath parameters
            opts = nepath.NonEquidistantOptions()
            opts.delta = delta
            opts.alpha = alpha
            opts.dot_delta = 1.0
            opts.ddot_delta = 0.1
            opts.optimize_Q = True
            opts.optimize_S = False
            opts.optimize_L = False
            opts.lambda_Q = 1.0
            opts.wash = True
            opts.washdis = washdis
            opts.washdis = washdis
            opts.connector = nepath.ConnectAlgorithm.none  # No connection
            opts.optimizer = nepath.OptimizationAlgorithm.ipopt  # Use IPOPT solver

            # Generate IQOP paths with DFS
            IQOP_paths = planner.IQOP(opts, False)  # log=False for cleaner output
            print(f"There are {len(IQOP_paths)} continuous toolpaths in total.")

            # Write output files
            clear_mkdir(output_dir / "path_IQOP_Ipopt_DFS/")
            write_csv_batch(IQOP_paths, output_dir / "path_IQOP_Ipopt_DFS/", ".csv")

            return IQOP_paths

        def demo_IQOP_gurobi(
            delta: float = 1.0,
            alpha: float = 0.5,
            washdis: float = 0.2,
            output_dir: str = "./data_examples/demo_IQOP_gurobi/",
        ) -> list[Path]:
            """
            Generate IQOP toolpaths based on gurobi.

            Parameters
            ----------
            delta : float
                Upper bound of delta_i (line width)
            alpha : float
                Scale of minimum distance
            washdis : float
                Resampling distance for uniform waypoint distribution
            output_dir : str
                Directory to save output files

            Returns
            -------
            list[Path]
                List of IQOP toolpath objects
            """
            planner = nepath.NEPathPlanner()

            # Create contour (slightly different for IQOP)
            theta = np.linspace(0, 2 * np.pi, 1000)
            r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
            x = r * np.cos(theta)
            y = r * np.sin(theta)

            # Tool compensate to get the outmost toolpath
            planner_toolcompensate = nepath.NEPathPlanner()
            planner_toolcompensate.set_contour(x, y)
            opts_toolcompensate = nepath.ContourParallelOptions()
            opts_toolcompensate.delta = -0.5 * delta
            opts_toolcompensate.wash = True
            opts_toolcompensate.washdis = washdis
            path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

            planner.set_contour(path_outmost[0])

            # Set toolpath parameters
            opts = nepath.NonEquidistantOptions()
            opts.delta = delta
            opts.alpha = alpha
            opts.dot_delta = 1.0
            opts.ddot_delta = 0.1
            opts.optimize_Q = True
            opts.optimize_S = False
            opts.optimize_L = False
            opts.lambda_Q = 1.0
            opts.wash = True
            opts.washdis = washdis
            opts.connector = nepath.ConnectAlgorithm.none  # No connection
            opts.optimizer = nepath.OptimizationAlgorithm.gurobi  # Use gurobi solver

            # Generate IQOP paths
            IQOP_paths = planner.IQOP(opts, False)  # log=False for cleaner output
            print(f"There are {len(IQOP_paths)} continuous toolpaths in total.")

            # Write output files
            clear_mkdir(output_dir / "paths_gurobi_IQ/")
            write_csv_batch(IQOP_paths, output_dir / "paths_gurobi_IQ/", ".csv")
            contour_path = nepath.Path.from_arrays(x, y)
            write_csv(contour_path, output_dir / "contour.csv")

            return IQOP_paths

        def demo_IQOP_gurobi_CFS(
            delta: float = 1.0,
            alpha: float = 0.5,
            washdis: float = 0.2,
            output_dir: str = "./data_examples/demo_IQOP_gurobi/",
        ) -> list[Path]:
            """
            Generate IQOP toolpaths with CFS (Connected Fermat Spiral) connection based on gurobi.

            Parameters
            ----------
            delta : float
                Upper bound of delta_i (line width)
            alpha : float
                Scale of minimum distance
            washdis : float
                Resampling distance for uniform waypoint distribution
            output_dir : str
                Directory to save output files

            Returns
            -------
            list[Path]
                List of IQOP toolpath objects with CFS connection
            """
            planner = nepath.NEPathPlanner()

            # Create contour (slightly different for IQOP)
            theta = np.linspace(0, 2 * np.pi, 1000)
            r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
            x = r * np.cos(theta)
            y = r * np.sin(theta)

            # Tool compensate to get the outmost toolpath
            planner_toolcompensate = nepath.NEPathPlanner()
            planner_toolcompensate.set_contour(x, y)
            opts_toolcompensate = nepath.ContourParallelOptions()
            opts_toolcompensate.delta = -0.5 * delta
            opts_toolcompensate.wash = True
            opts_toolcompensate.washdis = washdis
            path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

            planner.set_contour(path_outmost[0])

            # Set toolpath parameters
            opts = nepath.NonEquidistantOptions()
            opts.delta = delta
            opts.alpha = alpha
            opts.dot_delta = 1.0
            opts.ddot_delta = 0.1
            opts.optimize_Q = True
            opts.optimize_S = False
            opts.optimize_L = False
            opts.lambda_Q = 1.0
            opts.wash = True
            opts.washdis = washdis
            opts.connector = nepath.ConnectAlgorithm.none  # No connection
            opts.optimizer = nepath.OptimizationAlgorithm.gurobi  # Use gurobi solver

            # Generate IQOP paths with CFS
            IQOP_paths = planner.IQOP(opts, False)  # log=False for cleaner output
            print(f"There are {len(IQOP_paths)} continuous toolpaths in total.")

            # Write output files
            clear_mkdir(output_dir / "path_IQOP_gurobi_CFS/")
            write_csv_batch(IQOP_paths, output_dir / "path_IQOP_gurobi_CFS/", ".csv")

            return IQOP_paths

        def demo_IQOP_gurobi_DFS(
            delta: float = 1.0,
            alpha: float = 0.5,
            washdis: float = 0.2,
            output_dir: str = "./data_examples/demo_IQOP_gurobi/",
        ) -> list[Path]:
            """
            Generate IQOP toolpaths with DFS (Depth First Search) connection based on gurobi.

            Parameters
            ----------
            delta : float
                Upper bound of delta_i (line width)
            alpha : float
                Scale of minimum distance
            washdis : float
                Resampling distance for uniform waypoint distribution
            output_dir : str
                Directory to save output files

            Returns
            -------
            list[Path]
                List of IQOP toolpath objects with DFS connection
            """
            planner = nepath.NEPathPlanner()

            # Create contour (slightly different for IQOP)
            theta = np.linspace(0, 2 * np.pi, 1000)
            r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
            x = r * np.cos(theta)
            y = r * np.sin(theta)

            # Tool compensate to get the outmost toolpath
            planner_toolcompensate = nepath.NEPathPlanner()
            planner_toolcompensate.set_contour(x, y)
            opts_toolcompensate = nepath.ContourParallelOptions()
            opts_toolcompensate.delta = -0.5 * delta
            opts_toolcompensate.wash = True
            opts_toolcompensate.washdis = washdis
            path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

            planner.set_contour(path_outmost[0])

            # Set toolpath parameters
            opts = nepath.NonEquidistantOptions()
            opts.delta = delta
            opts.alpha = alpha
            opts.dot_delta = 1.0
            opts.ddot_delta = 0.1
            opts.optimize_Q = True
            opts.optimize_S = False
            opts.optimize_L = False
            opts.lambda_Q = 1.0
            opts.wash = True
            opts.washdis = washdis
            opts.washdis = washdis
            opts.connector = nepath.ConnectAlgorithm.none  # No connection
            opts.optimizer = nepath.OptimizationAlgorithm.gurobi  # Use gurobi solver

            # Generate IQOP paths with DFS
            IQOP_paths = planner.IQOP(opts, False)  # log=False for cleaner output
            print(f"There are {len(IQOP_paths)} continuous toolpaths in total.")

            # Write output files
            clear_mkdir(output_dir / "path_IQOP_gurobi_DFS/")
            write_csv_batch(IQOP_paths, output_dir / "path_IQOP_gurobi_DFS/", ".csv")

            return IQOP_paths

except (AttributeError, ImportError):
    pass  # IQOP not available
