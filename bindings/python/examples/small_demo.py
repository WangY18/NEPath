import numpy as np
import NEPath as nepath

if __name__ == "__main__":
    # Create contour (slightly different for IQOP)
    theta = np.linspace(0, 2 * np.pi, 1000)
    r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # Set parameters
    delta = 1.0
    washdis = 0.2
    radius = 1.0
    threshold = 0.3

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    # Initialize NEPath planner and set contour
    planner = nepath.NEPathPlanner()
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
