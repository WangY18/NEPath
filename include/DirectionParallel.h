#pragma once
#include "Curve.h"
#include "clipper.hpp"
// using namespace ClipperLib;
// DirectionParalle is a class to plan Raster and Zigzag toolpaths.

namespace nepath
{
    class DirectionParalle
    {
    public:
        static paths Raster(const path &contour, const paths &holes, double dis, double angle = 0); // Generate Raster toolpaths
        static paths Zigzag(const path &contour, const paths &holes, double dis, double angle = 0); // Generate Zigzag toolpaths
    private:
        static ClipperLib::Paths Raster(const ClipperLib::Path &contour, const ClipperLib::Paths &holes, double dis, double scale); // Generate Raster toolpaths in Path form
        static bool cmp_Raster(const ClipperLib::IntPoint &a, const ClipperLib::IntPoint &b);                                       // comparasion between points in Raster
        static ClipperLib::Paths Zigzag(const ClipperLib::Path &contour, const ClipperLib::Paths &holes, double dis, double scale); // Generate Zigzag toolpaths in Path form
    };
}