#pragma once
// DirectParallelOptions is a struct to store information of parameters in Raster and Zigzag toolpaths.
// ContourParallelOptions is a struct to store information of parameters in CP toolpaths.

struct DirectParallelOptions {
	double delta = 1.5; // the line width
	double angle = 0.0; // the angle (rad) between toolpaths and the x-axis.
};

struct ContourParallelOptions {
	double delta = 1.5; // the line width
	bool wash = true;
	double washdis = 0.2;
	int num_least = 50;
	// If wash==true, the toolpaths would be resampled with a uniformly-distributed distance no more than wash_dis, and the number of waypoints are no less than num_least.
};