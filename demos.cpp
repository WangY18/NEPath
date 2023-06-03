#include "demos.h"
#include "NEPath-master/NEPathPlanner.h"
#include "NEPath-master/FileAgent.h"
#include "NEPath-master/NonEquidistant.h"
#include <cmath>

void demo_Raster() {
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
	opts.angle = - pi / 3.0; // the angle of raster toolpaths, unit: rad

	paths raster_paths = planner.Raster(opts); // all raster paths
	cout << "There are " << raster_paths.size() << " continuous toolpaths in total." << endl;

	FileAgent::delete_AllFiles(R"(.\data_examples\demo_raster\)");
	FileAgent::write_csv(raster_paths, R"(.\data_examples\demo_raster\)", ".csv");
	//FileAgent::write_csv(contour, R"(.\data_examples\contour.csv)");
}

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

void demo_CP() {
	NEPathPlanner planner;

	// Obtain the contour of the outer boundary of slices
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

	// The out boundary should be offset with half of the line width to obtain the outmost toolpath
	NEPathPlanner planner_toolcompensate;
	planner_toolcompensate.set_contour(contour);
	ContourParallelOptions opts_toolcompensate;
	opts_toolcompensate.delta = -1.0 * 0.5; // half of the line width of toolpaths
	opts_toolcompensate.wash = true; // it is recommended to set opt.wash=true
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
	for (int i = 0; i < CP_paths.size(); ++i) {
		// CP_paths[i] is the i-th continuous toolpath
		cout << "Toopath " << i << " has " << CP_paths[i].length << " waypoints." << endl;
	}

	FileAgent::delete_AllFiles(R"(.\data_examples\demo_CP\)");
	FileAgent::write_csv(CP_paths, R"(.\data_examples\demo_CP\)", ".csv");
	// FileAgent::write_csv(contour, R"(.\data_examples\contour.csv)");
}

void demo_tool_compensate() {
	NEPathPlanner planner;

	// Obtain the contour of the outer boundary of slices
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

	// Obtain the hole
	double x_hole[] = { -5,5,5,0,-5 };
	double y_hole[] = { -5,-5,5,0,5 };
	planner.addhole(x_hole, y_hole, 5);

	// Tool compensate
	ContourParallelOptions opts;
	opts.delta = -1.5; // the offset distance
	opts.wash = true; // it is recommended to set opt.wash=true
	// if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
	opts.washdis = 0.2;
	paths ps_toolcompensate = planner.tool_compensate(opts); // Tool compensate

	cout << "There are " << ps_toolcompensate.size() << " continuous toolpaths in total." << endl;
	for (int i = 0; i < ps_toolcompensate.size(); ++i) {
		// ps_toolcompensate[i] is the i-th continuous toolpath
		cout << "Toopath " << i << " has " << ps_toolcompensate[i].length << " waypoints." << endl;
	}

	FileAgent::delete_AllFiles(R"(.\data_examples\demo_toolcompensate\)");
	FileAgent::write_csv(ps_toolcompensate, R"(.\data_examples\demo_toolcompensate\)", ".csv");
	// FileAgent::write_csv(contour, R"(.\data_examples\contour.csv)");
	FileAgent::write_csv(planner.holes[0], R"(.\data_examples\hole.csv)");
}

void demo_underfill() {
	NEPathPlanner planner;

	// Obtain the contour of the outer boundary of slices
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

	// The out boundary should be offset with half of the line width to obtain the outmost toolpath
	NEPathPlanner planner_toolcompensate;
	planner_toolcompensate.set_contour(contour);
	ContourParallelOptions opts_toolcompensate;
	opts_toolcompensate.delta = -1.0 * 0.5; // half of the line width of toolpaths
	opts_toolcompensate.wash = true; // it is recommended to set opt.wash=true
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
	double reratio = 0.03; // resolution ratio for underfill computation

	UnderFillSolution ufs = Curve::UnderFill(contour, paths(), CP_paths, delta_underfill, reratio); // Obtain the results of underfill

	cout << "The underfill rate is " << ufs.underfillrate * 100 << "%." << endl;

	// Output as files
	std::ofstream xsoutFile(R"(.\data_examples\demo_underfill\underfill\xs.csv)", std::ios::out);
	xsoutFile << "x\n";
	for (int i = 0; i < ufs.nx; ++i) {
		xsoutFile << ufs.xs[i] << '\n';
	}
	xsoutFile.close();

	std::ofstream ysoutFile(R"(.\data_examples\demo_underfill\underfill\ys.csv)", std::ios::out);
	ysoutFile << "y\n";
	for (int i = 0; i < ufs.ny; ++i) {
		ysoutFile << ufs.ys[i] << '\n';
	}
	ysoutFile.close();

	std::ofstream mapsliceoutFile(R"(.\data_examples\demo_underfill\underfill\map_slice.csv)", std::ios::out);
	for (int i = 0; i < ufs.nx; ++i) {
		for (int j = 0; j < ufs.ny - 1; ++j) {
			mapsliceoutFile << ufs.map_slice[i][j] << ',';
		}
		mapsliceoutFile << ufs.map_slice[i][ufs.ny - 1] << '\n';
	}
	mapsliceoutFile.close();

	std::ofstream mapdeltaoutFile(R"(.\data_examples\demo_underfill\underfill\map_delta.csv)", std::ios::out);
	for (int i = 0; i < ufs.nx; ++i) {
		for (int j = 0; j < ufs.ny - 1; ++j) {
			mapdeltaoutFile << ufs.map_delta[i][j] << ',';
		}
		mapdeltaoutFile << ufs.map_delta[i][ufs.ny - 1] << '\n';
	}
	mapdeltaoutFile.close();

	FileAgent::delete_AllFiles(R"(.\data_examples\demo_underfill\paths\)");
	FileAgent::write_csv(CP_paths, R"(.\data_examples\demo_underfill\paths\)", ".csv");
	// FileAgent::write_csv(contour, R"(.\data_examples\contour.csv)");
}

void demo_sharpcorner() {
	NEPathPlanner planner;

	// Obtain the contour of the outer boundary of slices
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

	// The out boundary should be offset with half of the line width to obtain the outmost toolpath
	NEPathPlanner planner_toolcompensate;
	planner_toolcompensate.set_contour(contour);
	ContourParallelOptions opts_toolcompensate;
	opts_toolcompensate.delta = -1.0 * 0.5; // half of the line width of toolpaths
	opts_toolcompensate.wash = true; // it is recommended to set opt.wash=true
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

	double radius = 1.0; // radius of the rolling circle
	double threshold = 0.3; // threshold of area on one side to determine a sharp corner

	// Obtain the results of underfill
	int num = 0;
	paths ps_sharpturn;
	for (int i = 0; i < CP_paths.size(); ++i) {
		path p = path();
		SharpTurnSolution sol = Curve::SharpTurn_Invariant(CP_paths[i], radius, threshold, true, 0.5);
		for (int j = 0; j < sol.length; ++j) {
			num += sol.SharpTurn[j];
		}
		p.length = sol.length;
		p.x = new double[p.length];
		p.y = new double[p.length];
		for (int j = 0; j < p.length; ++j) {
			p.x[j] = sol.SharpTurn[j];
			p.y[j] = sol.AreaPercent[j];
		}
		ps_sharpturn.push_back(p);
	}

	cout << "There exist " << num << " sharp corners." << endl;

	// Output as files
	FileAgent::delete_AllFiles(R"(.\data_examples\demo_sharpcorner\sharpcorner\)");
	FileAgent::write_csv(ps_sharpturn, R"(.\data_examples\demo_sharpcorner\sharpcorner\)", ".csv");
	FileAgent::delete_AllFiles(R"(.\data_examples\demo_sharpcorner\paths\)");
	FileAgent::write_csv(CP_paths, R"(.\data_examples\demo_sharpcorner\paths\)", ".csv");
	// FileAgent::write_csv(contour, R"(.\data_examples\contour.csv)");
}

void demo_IQOP() {
	NEPathPlanner planner;

	// Obtain the contour of the outer boundary of slices
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

	// The out boundary should be offset with half of the line width to obtain the outmost toolpath
	NEPathPlanner planner_toolcompensate;
	planner_toolcompensate.set_contour(contour);
	ContourParallelOptions opts_toolcompensate;
	opts_toolcompensate.delta = -1.0 * 0.5; // half of the line width of toolpaths
	opts_toolcompensate.wash = true; // it is recommended to set opt.wash=true
	// if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
	opts_toolcompensate.washdis = 0.2;
	paths path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate);

	// planner.set_contour(path_outmost[0]);
	// or `planner.set_contour(contour.x, contour.y, contour.length)`

	// Set the toolpath parameters
	NonEquidistantOptions opts;
	opts.delta = 2.0; // the line width of toolpaths
	opts.alpha = 0.0;
	opts.dot_delta = 1.0; // the upper bound of \dot{delta_i}
	opts.ddot_delta = 0.1; // the upper bound of \ddot{delta_i}

	opts.optimize_Q = true; // the isoperimetric quotient is in the objective function
	opts.optimize_S = true; // the area is not in the objective function
	opts.optimize_L = true; // the length is not in the objective function
	opts.lambda_Q = 1.0; // the weighting coefficient of the isoperimetric quotient

	opts.wash = true; // it is recommended to set opt.wash=true
	// if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
	opts.washdis = 0.2;

	NonEquidistant NE(true);

	paths IQOP_paths = NE.NEpaths(path_outmost[0], paths(), opts); // all IQOP paths
	cout << "There are " << IQOP_paths.size() << " continuous toolpaths in total." << endl;

	FileAgent::delete_AllFiles(R"(.\data_examples\demo_IQOP\)");
	FileAgent::write_csv(IQOP_paths, R"(.\data_examples\demo_IQOP\)", ".csv");

	/*
	struct NonEquidistantOptions {
	double delta = 1.0; // the upper bound of delta_i
	double alpha = 0.0; // the lower bound of delta_i
	double dot_delta = 1.0; // the upper bound of \dot{delta_i}
	double ddot_delta = 0.1; // the upper bound of \ddot{delta_i}

	bool wash = true;
	double washdis = 0.2;
	int num_least = 50;
	// If wash==true, the toolpaths would be resampled with a uniformly-distributed distance no more than wash_dis, and the number of waypoints are no less than num_least.

	bool optimize_Q = true; // whether the isoperimetric quotient is in the objective function
	bool optimize_S = true; // whether the area is in the objective function
	bool optimize_L = true; // whether the length is in the objective function
	double lambda_Q = 1.0; // the weighting coefficient of the isoperimetric quotient
	double lambda_S = 1.0; // the weighting coefficient of the area
	double lambda_L = 1.0; // the weighting coefficient of the length

	double epsilon = 1e-2; // the maximum error of offsetting distances
	int step_max = 10; // the maximum iteration steps

	// TODO clear void
};
	*/
}