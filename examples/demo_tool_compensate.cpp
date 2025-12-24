#include <NEPath/NEPathPlanner.h>
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
        double r = 15.0 * (1.0 + 0.15 * cos(10.0 * theta));
        contour.x[i] = r * cos(theta);
        contour.y[i] = r * sin(theta);
    }
    planner.set_contour(contour);

    // Obtain the hole
    double x_hole[] = {-5, 5, 5, 0, -5};
    double y_hole[] = {-5, -5, 5, 0, 5};
    planner.addhole(x_hole, y_hole, 5);

    // Tool compensate
    ContourParallelOptions opts;
    opts.delta = -1.5; // the offset distance
    opts.wash = true;  // it is recommended to set opt.wash=true
    // if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
    opts.washdis = 0.2;
    paths ps_toolcompensate = planner.tool_compensate(opts); // Tool compensate

    cout << "Tool Compensate: There are " << ps_toolcompensate.size() << " continuous toolpaths in total." << endl;
    for (int i = 0; i < ps_toolcompensate.size(); ++i)
    {
        // ps_toolcompensate[i] is the i-th continuous toolpath
        cout << "Toopath " << i << " has " << ps_toolcompensate[i].length << " waypoints." << endl;
    }

    FileAgent::mkdir((fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "demo_toolcompensate").string().c_str(), true);
    FileAgent::write_csv(ps_toolcompensate, (fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "demo_toolcompensate" / "").string().c_str(), ".csv");
    // // FileAgent::write_csv(contour, (fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "contour.csv").string().c_str());
    FileAgent::write_csv(planner.holes[0], (fs::path(__FILE__).parent_path().parent_path() / "data_examples" / "hole.csv").string().c_str());

    return 0;
}