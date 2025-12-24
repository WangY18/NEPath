#include "NEPathPlanner.h"
#include "FileAgent.h"
#include <iostream>
#include <filesystem>
namespace fs = std::filesystem;
using namespace std;
using namespace nepath;

int main()
{
    NEPathPlanner planner;

    // Set the contour
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
    planner.set_contour(contour);
    // or `planner.set_contour(contour.x, contour.y, contour.length)`

    // Set the toolpath parameters
    DirectParallelOptions opts;
    opts.delta = 1.0;       // the line width of toolpaths
    opts.angle = -pi / 3.0; // the angle of raster toolpaths, unit: rad

    paths raster_paths = planner.Raster(opts); // all raster paths
    cout << "Raster: There are " << raster_paths.size() << " continuous toolpaths in total." << endl;

    FileAgent::mkdir((fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "demo_raster").string().c_str(), true);
    FileAgent::write_csv(raster_paths, (fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "demo_raster" / "").string().c_str(), ".csv");
    // FileAgent::write_csv(contour, (fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "demo_raster" / "contour.csv").string().c_str());

    return 0;
}