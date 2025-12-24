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

    double delta_underfill = opts.delta; // the line width for underfill computation
    double reratio = 0.03;               // resolution ratio for underfill computation

    UnderFillSolution ufs = Curve::UnderFill(contour, paths(), CP_paths, delta_underfill, reratio); // Obtain the results of underfill

    cout << "The underfill rate is " << ufs.underfillrate * 100 << "%." << endl;

    // Output as files
    std::ofstream xsoutFile((fs::path(__FILE__).parent_path() / "data_examples" / "demo_underfill" / "underfill" / "xs.csv").string(), std::ios::out);
    xsoutFile << "x\n";
    for (int i = 0; i < ufs.nx; ++i)
    {
        xsoutFile << ufs.xs[i] << '\n';
    }
    xsoutFile.close();

    std::ofstream ysoutFile((fs::path(__FILE__).parent_path() / "data_examples" / "demo_underfill" / "underfill" / "ys.csv").string(), std::ios::out);
    ysoutFile << "y\n";
    for (int i = 0; i < ufs.ny; ++i)
    {
        ysoutFile << ufs.ys[i] << '\n';
    }
    ysoutFile.close();

    std::ofstream mapsliceoutFile((fs::path(__FILE__).parent_path() / "data_examples" / "demo_underfill" / "underfill" / "map_slice.csv").string(), std::ios::out);
    for (int i = 0; i < ufs.nx; ++i)
    {
        for (int j = 0; j < ufs.ny - 1; ++j)
        {
            mapsliceoutFile << ufs.map_slice[i][j] << ',';
        }
        mapsliceoutFile << ufs.map_slice[i][ufs.ny - 1] << '\n';
    }
    mapsliceoutFile.close();

    std::ofstream mapdeltaoutFile((fs::path(__FILE__).parent_path() / "data_examples" / "demo_underfill" / "underfill" / "map_delta.csv").string(), std::ios::out);
    for (int i = 0; i < ufs.nx; ++i)
    {
        for (int j = 0; j < ufs.ny - 1; ++j)
        {
            mapdeltaoutFile << ufs.map_delta[i][j] << ',';
        }
        mapdeltaoutFile << ufs.map_delta[i][ufs.ny - 1] << '\n';
    }
    mapdeltaoutFile.close();

    FileAgent::mkdir((fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "demo_underfill" / "paths").string().c_str(), true);
    FileAgent::write_csv(CP_paths, (fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "demo_underfill" / "paths" / "").string().c_str(), ".csv");
    // FileAgent::write_csv(contour, (fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "contour.csv").string().c_str());

    return 0;
}