"""
NEPath Demo Functions

This module replicates the functionality from demos.cpp using the Python bindings.
"""

import numpy as np
# from . import _nepath
import nepath_bindings as _nepath


def create_contour(num_points=1000):
    """Create a sample contour (flower shape)."""
    theta = np.linspace(0, 2 * np.pi, num_points)
    r = 15.0 * (1.0 + 0.15 * np.cos(10.0 * theta))
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y


def demo_Raster(delta=1.0, angle=-np.pi/3.0, output_dir="./data_examples/demo_raster/"):
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
    """
    planner = _nepath.NEPathPlanner()

    # Set the contour
    x, y = create_contour()
    planner.set_contour(x, y)

    # Set toolpath parameters
    opts = _nepath.DirectParallelOptions()
    opts.delta = delta
    opts.angle = angle

    # Generate raster paths
    raster_paths = planner.Raster(opts)
    print(f"There are {len(raster_paths)} continuous toolpaths in total.")

    # Write output files
    _nepath.FileAgent.delete_AllFiles(output_dir)
    _nepath.FileAgent.write_csv(raster_paths, output_dir, ".csv")

    return raster_paths


def demo_Zigzag(delta=1.0, angle=np.pi/3.0, output_dir="./data_examples/demo_zigzag/"):
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
    """
    planner = _nepath.NEPathPlanner()

    # Set the contour
    x, y = create_contour()
    planner.set_contour(x, y)

    # Set toolpath parameters
    opts = _nepath.DirectParallelOptions()
    opts.delta = delta
    opts.angle = angle

    # Generate zigzag paths
    zigzag_paths = planner.Zigzag(opts)
    print(f"There are {len(zigzag_paths)} continuous toolpaths in total.")
    for i, path in enumerate(zigzag_paths):
        print(f"Toolpath {i} has {path.length} waypoints.")

    # Write output files
    _nepath.FileAgent.delete_AllFiles(output_dir)
    _nepath.FileAgent.write_csv(zigzag_paths, output_dir, ".csv")

    # Write contour
    contour_path = _nepath.Path.from_arrays(x, y)
    _nepath.FileAgent.write_csv(contour_path, "./data_examples/contour.csv")

    return zigzag_paths


def demo_CP(delta=1.0, washdis=0.2, output_dir="./data_examples/demo_CP/"):
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
    """
    planner = _nepath.NEPathPlanner()

    # Create contour
    x, y = create_contour()

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = _nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = _nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = _nepath.ContourParallelOptions()
    opts.delta = delta
    opts.wash = True
    opts.washdis = washdis

    # Generate CP paths
    CP_paths = planner.CP(opts)
    print(f"There are {len(CP_paths)} continuous toolpaths in total.")
    for i, path in enumerate(CP_paths):
        print(f"Toolpath {i} has {path.length} waypoints.")

    # Write output files
    _nepath.FileAgent.delete_AllFiles(output_dir)
    _nepath.FileAgent.write_csv(CP_paths, output_dir, ".csv")

    return CP_paths


def demo_CP_CFS(delta=1.0, washdis=0.2, output_dir="./data_examples/demo_CP_CFS/"):
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
    """
    planner = _nepath.NEPathPlanner()

    # Create contour
    x, y = create_contour()

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = _nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = _nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = _nepath.ContourParallelOptions()
    opts.delta = delta
    opts.wash = True
    opts.washdis = washdis
    opts.connect = _nepath.ConnectAlgorithm.cfs  # Use CFS connection

    # Generate CP paths
    CP_paths = planner.CP(opts)
    print(f"There are {len(CP_paths)} continuous toolpaths in total.")
    for i, path in enumerate(CP_paths):
        print(f"Toolpath {i} has {path.length} waypoints.")

    # Write output files
    _nepath.FileAgent.delete_AllFiles(output_dir)
    _nepath.FileAgent.write_csv(CP_paths, output_dir, ".csv")

    return CP_paths


def demo_CP_DFS(delta=1.0, washdis=0.2, output_dir="./data_examples/demo_CP_DFS/"):
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
    """
    planner = _nepath.NEPathPlanner()

    # Create contour
    x, y = create_contour()

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = _nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = _nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = _nepath.ContourParallelOptions()
    opts.delta = delta
    opts.wash = True
    opts.washdis = washdis
    opts.connect = _nepath.ConnectAlgorithm.dfs  # Use DFS connection

    # Generate CP paths
    CP_paths = planner.CP(opts)
    print(f"There are {len(CP_paths)} continuous toolpaths in total.")
    for i, path in enumerate(CP_paths):
        print(f"Toolpath {i} has {path.length} waypoints.")

    # Write output files
    _nepath.FileAgent.delete_AllFiles(output_dir)
    _nepath.FileAgent.write_csv(CP_paths, output_dir, ".csv")

    return CP_paths


def demo_tool_compensate(delta_offset=-1.5, washdis=0.2, output_dir="./data_examples/demo_toolcompensate/"):
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
    """
    planner = _nepath.NEPathPlanner()

    # Create contour
    x, y = create_contour()
    planner.set_contour(x, y)

    # Add a hole
    x_hole = np.array([-5, 5, 5, 0, -5], dtype=float)
    y_hole = np.array([-5, -5, 5, 0, 5], dtype=float)
    planner.addhole(x_hole, y_hole)

    # Tool compensate
    opts = _nepath.ContourParallelOptions()
    opts.delta = delta_offset
    opts.wash = True
    opts.washdis = washdis
    ps_toolcompensate = planner.tool_compensate(opts)

    print(f"There are {len(ps_toolcompensate)} continuous toolpaths in total.")
    for i, path in enumerate(ps_toolcompensate):
        print(f"Toolpath {i} has {path.length} waypoints.")

    # Write output files
    _nepath.FileAgent.delete_AllFiles(output_dir)
    _nepath.FileAgent.write_csv(ps_toolcompensate, output_dir, ".csv")
    _nepath.FileAgent.write_csv(planner.holes[0], "./data_examples/hole.csv")

    return ps_toolcompensate


def demo_underfill(delta=1.0, washdis=0.2, reratio=0.03, output_dir="./data_examples/demo_underfill/"):
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
    """
    planner = _nepath.NEPathPlanner()

    # Create contour
    x, y = create_contour()

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = _nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = _nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = _nepath.ContourParallelOptions()
    opts.delta = delta
    opts.wash = True
    opts.washdis = washdis

    # Generate CP paths
    CP_paths = planner.CP(opts)
    print(f"There are {len(CP_paths)} continuous toolpaths in total.")

    # Calculate underfill
    contour_path = _nepath.Path.from_arrays(x, y)
    ufs = _nepath.Curve.UnderFill(contour_path, [], CP_paths, delta, reratio)
    print(f"The underfill rate is {ufs.underfillrate * 100:.2f}%.")

    # Write output files
    _nepath.FileAgent.delete_AllFiles(output_dir + "paths/")
    _nepath.FileAgent.write_csv(CP_paths, output_dir + "paths/", ".csv")

    # Note: For underfill maps, you would need to write custom code to save xs, ys, map_slice, map_delta
    # as these require special handling in Python

    return CP_paths, ufs


def demo_sharpcorner(delta=1.0, washdis=0.2, radius=1.0, threshold=0.3,
                    output_dir="./data_examples/demo_sharpcorner/"):
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
    """
    planner = _nepath.NEPathPlanner()

    # Create contour
    x, y = create_contour()

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = _nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = _nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = _nepath.ContourParallelOptions()
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
        sol = _nepath.Curve.SharpTurn_Invariant(path, radius, threshold, True, 0.5)
        sharp_turn = sol.get_sharp_turn()
        num += sum(sharp_turn)

        # Store results (SharpTurn values as x, AreaPercent as y)
        area_percent = sol.get_area_percent()
        sharp_path = _nepath.Path()
        sharp_path.set_arrays(
            np.array([float(st) for st in sharp_turn], dtype=float),
            np.array(area_percent, dtype=float)
        )
        ps_sharpturn.append(sharp_path)

    print(f"There exist {num} sharp corners.")

    # Write output files
    _nepath.FileAgent.delete_AllFiles(output_dir + "sharpcorner/")
    _nepath.FileAgent.write_csv(ps_sharpturn, output_dir + "sharpcorner/", ".csv")
    _nepath.FileAgent.delete_AllFiles(output_dir + "paths/")
    _nepath.FileAgent.write_csv(CP_paths, output_dir + "paths/", ".csv")

    return CP_paths, ps_sharpturn


# IQOP demos (only if Gurobi is available)
try:
    # Test if IQOP is available
    test_planner = _nepath.NEPathPlanner()
    if hasattr(test_planner, 'IQOP'):
        def demo_IQOP(delta=1.0, alpha=0.5, washdis=0.2, output_dir="./data_examples/demo_IQOP/"):
            """
            Generate IQOP toolpaths.

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
            """
            planner = _nepath.NEPathPlanner()

            # Create contour (slightly different for IQOP)
            theta = np.linspace(0, 2 * np.pi, 1000)
            r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
            x = r * np.cos(theta)
            y = r * np.sin(theta)

            # Tool compensate to get the outmost toolpath
            planner_toolcompensate = _nepath.NEPathPlanner()
            planner_toolcompensate.set_contour(x, y)
            opts_toolcompensate = _nepath.ContourParallelOptions()
            opts_toolcompensate.delta = -0.5 * delta
            opts_toolcompensate.wash = True
            opts_toolcompensate.washdis = washdis
            path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

            planner.set_contour(path_outmost[0])

            # Set toolpath parameters
            opts = _nepath.NonEquidistantOptions()
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

            # Generate IQOP paths
            IQOP_paths = planner.IQOP(opts, True)
            print(f"There are {len(IQOP_paths)} continuous toolpaths in total.")

            # Write output files
            _nepath.FileAgent.delete_AllFiles(output_dir + "paths_IQ/")
            _nepath.FileAgent.write_csv(IQOP_paths, output_dir + "paths_IQ/", ".csv")
            contour_path = _nepath.Path.from_arrays(x, y)
            _nepath.FileAgent.write_csv(contour_path, output_dir + "contour.csv")

            return IQOP_paths
except:
    pass  # IQOP not available
