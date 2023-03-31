#pragma once
#include "Curve.h"
#include "clipper.hpp"
using namespace ClipperLib;

// 方向平行路径，包括Raster和Zigzag
class DirectionParalle { // TODO
public:
	static paths Raster(const path& contour, const paths& holes, double dis, double angle = 0); // 纯光栅算法
	static paths Zigzag(const path& contour, const paths& holes, double dis, double angle = 0); // 纯Zigzag算法
private:
	static Paths Raster(const Path& contour, const Paths& holes, double dis, double scale); // 光栅算法
	static bool cmp_Raster(const IntPoint& a, const IntPoint& b);
	static Paths Zigzag(const Path& contour, const Paths& holes, double dis, double scale); // 经典zigzag算法
};