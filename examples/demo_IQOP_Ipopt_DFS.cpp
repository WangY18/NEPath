#include <NEPath/NEPath.h>
#include <NEPath/FileAgent.h>
#include <iostream>
#include <filesystem>
namespace fs = std::filesystem;
using namespace std;
using namespace nepath;

int main()
{
    NEPathPlanner planner;

    // Obtain the contour of the outer boundary of slices
    path contour;
    contour.length = 1000;                    // the number of waypoints
    contour.x = new double[contour.length](); // x-coordinate of waypoints
    contour.y = new double[contour.length](); // y-coordinate of waypoints
    const double pi = acos(-1.0);             // pi == 3.1415926...
    for (int i = 0; i < contour.length; ++i)
    {
        double theta = 2.0 * pi * i / contour.length;
        double r = 15.0 * (1.0 + 0.1 * cos(10.0 * theta));
        contour.x[i] = r * cos(theta);
        contour.y[i] = r * sin(theta);
    }

    // The out boundary should be offset with half of the line width to obtain the outmost toolpath
    NEPathPlanner planner_toolcompensate;
    planner_toolcompensate.set_contour(contour);
    ContourParallelOptions opts_toolcompensate;
    opts_toolcompensate.delta = -1.0 * 0.5; // half of the line width of toolpaths
    opts_toolcompensate.wash = true;        // it is recommended to set opt.wash=true
    // if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
    opts_toolcompensate.washdis = 0.2;
    paths path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate);

    planner.set_contour(path_outmost[0]);
    // or `planner.set_contour(contour.x, contour.y, contour.length)`

    // Set the toolpath parameters
    NonEquidistantOptions opts;
    opts.delta = 1.0;      // the line width of toolpaths
    opts.alpha = 0.5;      // the scale of minimum distance
    opts.dot_delta = 1.0;  // the upper bound of \dot{delta_i}
    opts.ddot_delta = 0.1; // the upper bound of \ddot{delta_i}

    opts.optimize_Q = true;  // the isoperimetric quotient is in the objective function
    opts.optimize_S = false; // the area is not in the objective function
    opts.optimize_L = false; // the length is not in the objective function
    opts.lambda_Q = 1.0;     // the weighting coefficient of the isoperimetric quotient

    opts.wash = true; // it is recommended to set opt.wash=true
    // if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
    opts.washdis = 0.2;
    opts.connector = ConnectAlgorithm::dfs;        // select dfs as the connecting method
    opts.optimizer = OptimizationAlgorithm::ipopt; // use IPOPT solver

    paths IQOP_paths = planner.IQOP(opts, true); // IQOP with DFS
    cout << "There are " << IQOP_paths.size() << " continuous toolpaths in total." << endl;

    FileAgent::mkdir((fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "demo_IQOP_Ipopt_DFS" / "paths_IQ").string().c_str(), true);
    FileAgent::write_csv(IQOP_paths, (fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "demo_IQOP_Ipopt_DFS" / "paths_IQ" / "").string().c_str(), ".csv");
    // FileAgent::write_csv(contour, (fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "demo_IQOP_Ipopt_DFS" / "contour.csv").string().c_str());

    return 0;
}