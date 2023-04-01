#pragma once
#include "Curve.h"
#include "DirectionParallel.h"
#include "ContourParallel.h"
#include "PlanningOptions.h"

// 集大成——路径规划器
class NEPathPlanner {
public:
	NEPathPlanner();
	void set_contour(const double* x, const double* y, int length);
	void set_contour(const path& contour_new);
	void addhole(const path& hole);
	void addhole(const double* x, const double* y, int length);
	void addholes(const paths& holes_new);
	paths tool_compensate(double dis); // 刀补
	paths tool_compensate(const ContourParallelOptions& opts); // 刀补
public:
	paths Raster(double angle = 0); // 纯光栅算法；未复现轮廓平行混合的算法
	paths Raster(const DirectParallelOptions& opts); // 纯光栅算法；未复现轮廓平行混合的算法
	paths Zigzag(double angle = 0); // 纯Zigzag算法；未复现轮廓平行混合的算法
	paths Zigzag(const DirectParallelOptions& opts); // 纯Zigzag算法；未复现轮廓平行混合的算法
	paths CP(); // 轮廓平行算法
	paths CP(const ContourParallelOptions& opts); // 轮廓平行算法
public:
	path contour;
	paths holes;
	double delta;
	bool wash;
	double washdis;
	bool debug;
	int num_least;
};