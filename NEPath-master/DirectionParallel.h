#pragma once
#include "Curve.h"
#include "clipper.hpp"
using namespace ClipperLib;
// DirectionParalle is a class to plan Raster and Zigzag toolpaths.

class DirectionParalle {
public:
	static paths Raster(const path& contour, const paths& holes, double dis, double angle = 0); // Generate Raster toolpaths
	static paths Zigzag(const path& contour, const paths& holes, double dis, double angle = 0); // Generate Zigzag toolpaths
private:
	static Paths Raster(const Path& contour, const Paths& holes, double dis, double scale); // Generate Raster toolpaths in Path form
	static bool cmp_Raster(const IntPoint& a, const IntPoint& b); // comparasion between points in Raster
	static Paths Zigzag(const Path& contour, const Paths& holes, double dis, double scale); // Generate Zigzag toolpaths in Path form
};