#include "NEPathPlanner.h"
#include "setup_NEPath.h"
#include "FileAgent.h"
#include <cmath>
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
        double r = 15.0 * (1.0 + 0.15 * cos(10.0 * theta));
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
    ContourParallelOptions opts;
    opts.delta = 1.0; // the line width of toolpaths
    opts.wash = true; // it is recommended to set opt.wash=true
    // if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
    opts.washdis = 0.2;

    paths CP_paths = planner.CP(opts); // all CP paths
    cout << "There are " << CP_paths.size() << " continuous toolpaths in total." << endl;

    double radius = 1.0;    // radius of the rolling circle
    double threshold = 0.3; // threshold of area on one side to determine a sharp corner

    // Obtain the results of underfill
    int num = 0;
    paths ps_sharpturn;
    for (int i = 0; i < CP_paths.size(); ++i)
    {
        path p = path();
        SharpTurnSolution sol = Curve::SharpTurn_Invariant(CP_paths[i], radius, threshold, true, 0.5);
        for (int j = 0; j < sol.length; ++j)
        {
            num += sol.SharpTurn[j];
        }
        p.length = sol.length;
        p.x = new double[p.length];
        p.y = new double[p.length];
        for (int j = 0; j < p.length; ++j)
        {
            p.x[j] = sol.SharpTurn[j];
            p.y[j] = sol.AreaPercent[j];
        }
        ps_sharpturn.push_back(p);
    }

    cout << "There exist " << num << " sharp corners." << endl;

    // Output as files
    FileAgent::mkdir((fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "demo_sharpcorner" / "sharpcorner").string().c_str(), true);
    FileAgent::write_csv(ps_sharpturn, (fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "demo_sharpcorner" / "sharpcorner" / "").string().c_str(), ".csv");
    FileAgent::mkdir((fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "demo_sharpcorner" / "paths").string().c_str(), true);
    FileAgent::write_csv(CP_paths, (fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "demo_sharpcorner" / "paths" / "").string().c_str(), ".csv");
    // FileAgent::write_csv(contour, (fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "contour.csv").string().c_str());
    return 0;
}