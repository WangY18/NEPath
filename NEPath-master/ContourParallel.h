#pragma once
#include "Curve.h"
#include "clipper.hpp"
using namespace ClipperLib;


class ContourParallel {
public:
	static paths Contour_Parallel(const path& contour, const paths& holes, double dis, bool wash = true, double washdis = 0.5, int num_least = 50);
	static paths OffsetClipper(const double* x, const double* y, double dis, int length, bool wash = true, double washdis = 0.5, int num_least = 50); // 使用Clipper进行偏置，正的向内，负的向外
	static paths OffsetClipper(const path& contour, const paths& holes, double dis, bool wash = true, double washdis = 0.5, int num_least = 50); // 使用Clipper进行偏置并剪去holes，正的向内，负的向外
	static paths cut_holes(const path& contour, const paths& holes, bool wash = true, double washdis = 0.5, int num_least = 50);// 不相交的不被考虑
	static paths tool_compensate(const path& contour, const paths& holes, double dis, bool wash = true, double washdis = 0.5, int num_least = 50); // 刀补
	static paths set_minus(const path& contour, const paths& holes, bool onlyouter = false, bool wash = true, double washdis = 0.5, int num_least = 50); // 纯集合减法，无论是否
	
	static inline cInt double2cInt(const double& d, double scale, double delta_pos = 0.0);
	static inline double cInt2double(const cInt& c, double scale, double delta_pos = 0.0);
	static paths Paths2paths(const Paths& Ps, double scale, double delta_x = 0.0, double delta_y = 0.0);
	static path Path2path(const Path& P, double scale, double delta_x = 0.0, double delta_y = 0.0);
	static Path path2Path(const path& p, double scale, double delta_x = 0.0, double delta_y = 0.0);
	//static Paths tool_compensate(double dis);
	static void clearvoid(pathnode* root, const paths& holes, double delta, double area_err, double delta_errscale = 0.02); // 消除void
private:
	static pathnode* root_offset(const path& contour, const paths& holes, double dis, bool wash = true, double washdis = 0.5, int num_least = 50);
	static bool path_subset(const Path& p1, const Path& p2);
public:
	static const int ClipperBound = 1e8;
};