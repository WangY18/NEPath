#pragma once
#include "gurobi_c++.h"
#include "Curve.h"
#include "PlanningOptions.h"

class NonEquidistant {
public:
	NonEquidistant(bool debug = false);
	//paths NEpaths(const path& contour, const paths& holes, double delta, double dot_delta, double ddot_delta, double alpha, double err = 0.1, int N_maxiter = 5);
	paths NEpaths(const path& contour, const paths& holes, const NonEquidistantOptions& opts);
private:
	double* Optimize_QSL(path p, const NonEquidistantOptions& opts);
	double* L2A_optimize(double const* x, double const* y, int length, double delta, double dot_delta, double ddot_delta, double alpha, double* delta0 = NULL, double err = 0.1, int N_maxiter = 5); // 仅优化圆度，实现收敛性判别，但不考虑初始采样、路径光滑等。返回优化结果的deltas
	double* L_optimize(double const* x, double const* y, int length, double delta, double dot_delta, double ddot_delta, double alpha, double* delta0 = NULL, double err = 0.1, int N_maxiter = 5); // 仅优化长度
	double* IQA_optimize(double const* x, double const* y, int length, double delta, double dot_delta, double ddot_delta, double alpha, double lambda, double* delta0 = NULL, double err = 0.1, int N_maxiter = 5); // 优化圆度和面积
	double* QSL_optimize_outward(double const* x, double const* y, int length, double delta, double dot_delta, double ddot_delta, double lambdaQ, double lambdaS, double lambdaL, double err = 0.1, int N_maxiter = 10); // 边界平滑用的优化问题，圆度、面积和长度
	void addcons_ddot_deltas(GRBModel& model, double const* x, double const* y, GRBVar* deltas, int length, double dot_delta, double ddot_delta);
	// 进行一次路径规划，考虑初始采样、路径光滑等。
	paths do1offset(const path& contour, const NonEquidistantOptions& opts);
	pathnode* root_offset(const path& contour, const paths& holes, const NonEquidistantOptions& opts);
private:
	GRBEnv gurobi;
};