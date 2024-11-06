#pragma once
#include "setup_NEPath.h"
// NonEquidistant is a class to plan non-equidistant toolpaths.

#ifdef IncludeGurobi
#include "gurobi_c++.h"
#include "Curve.h"
#include "PlanningOptions.h"

class NonEquidistant {
public:
	NonEquidistant(bool debug = false);
	paths NEpaths(const path& contour, const paths& holes, const NonEquidistantOptions& opts);
	path NEpaths_CFS(const path& contour, const paths& holes, const NonEquidistantOptions& opts);
	path NEpaths_DFS(const path& contour, const paths& holes, const NonEquidistantOptions& opts);
private:
	double* Optimize_QSL(path p, const NonEquidistantOptions& opts);
	paths do1offset(const path& contour, const NonEquidistantOptions& opts);
	pathnode* root_offset(const path& contour, const paths& holes, const NonEquidistantOptions& opts);
private:
	GRBEnv gurobi;
};
#endif