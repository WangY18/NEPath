#pragma once
// DirectParallelOptions is a struct to store information of parameters in Raster and Zigzag toolpaths.
// ContourParallelOptions is a struct to store information of parameters in CP toolpaths.

namespace nepath
{
    enum ConnectAlgorithm
    {
        none,
        cfs, // Connector::ConnectedFermatSpiral_MultMinimum
        dfs  // Connector::ConnectedDFS
    };

    enum OptimizationAlgorithm
    {
        ipopt, // IPOPT solver
        gurobi // Gurobi solver
    };

    struct DirectParallelOptions
    {
        double delta = 1.0; // the line width
        double angle = 0.0; // the angle (rad) between toolpaths and the x-axis.
    };

    struct ContourParallelOptions
    {
        double delta = 1.0; // the line width
        bool wash = true;
        double washdis = 0.2;
        int num_least = 50;
        // If wash==true, the toolpaths would be resampled with a uniformly-distributed distance no more than wash_dis, and the number of waypoints are no less than num_least.

        ConnectAlgorithm connect = ConnectAlgorithm::none;
    };

    struct NonEquidistantOptions
    {
        double delta = 1.0;      // the upper bound of delta_i
        double alpha = 0.0;      // the lower bound of delta_i
        double dot_delta = 1.0;  // the upper bound of \dot{delta_i}
        double ddot_delta = 0.1; // the upper bound of \ddot{delta_i}

        bool optimize_Q = true; // whether the isoperimetric quotient is in the objective function
        bool optimize_S = true; // whether the area is in the objective function
        bool optimize_L = true; // whether the length is in the objective function
        double lambda_Q = 1.0;  // the weighting coefficient of the isoperimetric quotient
        double lambda_S = 1.0;  // the weighting coefficient of the area
        double lambda_L = 1.0;  // the weighting coefficient of the length

        double epsilon = 1e-2; // the maximum error of offsetting distances
        int step_max = 10;     // the maximum iteration steps

        bool wash = true;
        double washdis = 0.2;
        int num_least = 50;
        // If wash==true, the toolpaths would be resampled with a uniformly-distributed distance no more than wash_dis, and the number of waypoints are no less than num_least.

        ConnectAlgorithm connect = none;
    };
}