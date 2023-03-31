#pragma once
#include <iostream>
#include <fstream>
#include <stack>
#include <algorithm>
#include "Basic.h"
#include "path.h"

struct UnderFillSolution {
	bool** map_slice = NULL; // 在切片内
	bool** map_delta = NULL; // 在路径的delta邻域内
	int nx = -1;
	int ny = -1;
	double* xs = NULL;
	double* ys = NULL;
	double underfillrate = -1.0;
	void clear();
};

struct SharpTurnSolution {
	int length = 0; // 点的数量，等于路径长度，无论是否闭合
	double radius = -1.0; // 积分半径
	double* AreaPercent = NULL; // 各顶点处面积百分比，不闭合时约定AreaPercent[0], AreaPercent[end] = -1.0;
	double threshold = -1.0; // 百分比阈值
	bool* SharpTurn = NULL; // 急转弯标记，不闭合时约定SharpTurn[0], SharpTurn[end] = false;
	bool close = false; // 是否闭合
	void clear();
};

class Curve {
public: // 基础几何
	static double interp_id(const double* x, int length, double id);
	static double nearest_id(const double* x, const double* y, int length, double x0, double y0);
	static double distance_point2path(const double* x, const double* y, int length, double x0, double y0);
	static double furthest_id(const double* x, const double* y, int length, double x0, double y0);
	static double curves_nearest(const path& p, const path& q, double& id1, double& id2);
	static path* rotate(const path& p, double angle);
	static paths* rotate(const paths& ps, double angle);
	static bool interset_path(const path& a, const path& b);
	static bool interset_path_id(const path& a, const path& b, double& ida, double& idb);
	static void Ndir(const double* x, const double* y, int length, double*& nx, double*& ny); // 计算法向
	static double AreaCal(const double* x, const double* y, int length); // 计算面积
	static void DiffLength(double*& dl, const double* x, const double* y, int length, bool poly = true); // 计算长度
	static double* DiffLength(const double* x, const double* y, int length, bool poly = true); // 计算长度
	static double TotalLength(const double* x, const double* y, int length, bool poly = true); // 计算长度
	static double LengthBetween(const double* x, const double* y, int length, int from, int to); // 两点之间的长度
	static double LengthBetween(const double* x, const double* y, int length, double from, double to); // 两点之间的长度
	static void OffsetNaive(const double* x, const double* y, const double* delta, int length, double*& xnew, double*& ynew, double* nx = NULL, double* ny = NULL); // 简单偏置
	static void OffsetNaive(const double* x, const double* y, double delta, int length, double*& xnew, double*& ynew, double* nx = NULL, double* ny = NULL); // 简单偏置
public: // 评价指标相关
	static UnderFillSolution UnderFill(const path& contour, const paths& holes, const paths& ps, double delta, double reratio); // 欠填充计算结果
	static double UnderFillRate(const path& contour, const paths& holes, const paths& ps, double delta, double reratio); // 欠填充计算结果
	static SharpTurnSolution SharpTurn_Invariant(const path& p, double radius, double threshold = 0.3, bool close = false, double washdis = -1.0); // 急转弯计算
	static int SharpTurnNum_Invariant(const path& p, double radius, double threshold = 0.3, bool close = false, double washdis = -1.0); // 急转弯数量计算
private:
	static double AreaInvariant_OnePoint(const path& p, double radius, int id, bool close); // 计算一个点处的面积不变量
private:
	struct point {
		double x;
		double y;
		point(): x(0), y(0) {}
		point(double X, double Y) :x(X), y(Y) {}
		point(const point& p) :x(p.x), y(p.y) {}
	};
	static bool cmp_Raster(const point& a, const point& b);
public: // 数值计算相关
	static path wash_dis(const path& p, double dis);
	static path wash_dis(const double* x, const double* y, int length, double dis, bool output_poly = false);
public: // CFS相关
	static double BackDis(const double* x, const double* y, int length, double id, double dis);
	static double ForDis(const double* x, const double* y, int length, double id, double dis);
public: // 其他
	static void write_xy(const char* outfilename, const path& p);
};