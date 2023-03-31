#include "demos.h"
#include "NEPath-master/NEPathPlanner.h"
#include "NEPath-master/PlanningOptions.h"
#include "NEPath-master/FileAgent.h"
#include <cmath>

void demo_Zigzag() {
	NEPathPlanner planner;

	// Set the contour
	path contour;
	contour.length = 1000; // the number of waypoints
	contour.x = new double[contour.length](); // x-coordinate of waypoints
	contour.y = new double[contour.length](); // y-coordinate of waypoints
	const double pi = acos(-1.0); // pi == 3.1415926...
	for (int i = 0; i < contour.length; ++i) {
		double theta = 2.0 * pi * i / contour.length;
		double r = 15.0 * (1.0 + 0.15 * cos(10.0 * theta));
		contour.x[i] = r * cos(theta);
		contour.y[i] = r * sin(theta);
	}
	planner.set_contour(contour);
	// or `planner.set_contour(contour.x, contour.y, contour.length)`

	// Set the toolpath parameters
	DirectParallelOptions opts;
	opts.delta = 1.0; // the line width of toolpaths
	opts.angle = pi / 3.0; // the angle of Zigzag toolpaths, unit: rad

	paths zigzag_paths = planner.Zigzag(opts); // all zigzag paths
	cout << "There are " << zigzag_paths.size() << " continuous toolpaths in total." << endl;
	for (int i = 0; i < zigzag_paths.size(); ++i) {
		// zigzag_paths[i] is the i-th continuous toolpath
		cout << "Toopath " << i << " has " << zigzag_paths[i].length << " waypoints." << endl;
	}

	FileAgent::delete_AllFiles(R"(.\data_examples\demo_zigzag\)");
	FileAgent::write_csv(zigzag_paths, R"(.\data_examples\demo_zigzag\)", ".csv");
	FileAgent::write_csv(contour, R"(.\data_examples\contour.csv)");
}