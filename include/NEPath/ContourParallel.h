#pragma once
#include <NEPath/Curve.h>
#include "clipper.hpp"
// ContourParallel is a class to plan CP toolpaths.

namespace nepath
{
    class ContourParallel
    {
    public:
        static path Contour_Parallel_DFS(const path &contour, const paths &holes, double dis, bool wash = true, double washdis = 0.5, int num_least = 50);
        static path Contour_Parallel_CFS(const path &contour, const paths &holes, double dis, bool wash = true, double washdis = 0.5, int num_least = 50);
        static paths Contour_Parallel(const path &contour, const paths &holes, double dis, bool wash = true, double washdis = 0.5, int num_least = 50);      // Generate CP toolpath
        static paths OffsetClipper(const double *x, const double *y, double dis, int length, bool wash = true, double washdis = 0.5, int num_least = 50);    // Offset a path based on Clipper. dis>0 means offsetting the path inside; dis<0 means offsetting the path outside
        static paths cut_holes(const path &contour, const paths &holes, bool wash = true, double washdis = 0.5, int num_least = 50);                         // Apply set minus on contour to holes if intersections exist.
        static paths OffsetClipper(const path &contour, const paths &holes, double dis, bool wash = true, double washdis = 0.5, int num_least = 50);         // Offset the contour and apply set minus to holes if intersections exist based on Clipper. dis>0 means offsetting the path inside; dis<0 means offsetting the path outside
        static paths tool_compensate(const path &contour, const paths &holes, double dis, bool wash = true, double washdis = 0.5, int num_least = 50);       // Tool compensate. dis>0 means offsetting the path inside; dis<0 means offsetting the path outside
        static paths set_minus(const path &contour, const paths &holes, bool onlyouter = false, bool wash = true, double washdis = 0.5, int num_least = 50); // Apply set minus on contour to holes no matter whether intersections exist.
    public:
        // Transform from a double variable to a cInt variable
        static inline ClipperLib::cInt double2cInt(const double &d, double scale, double delta_pos = 0.0)
        {
            return (d - delta_pos) * scale;
        }
        // Transform from a cInt variable to a double variable
        static inline double cInt2double(const ClipperLib::cInt &c, double scale, double delta_pos = 0.0)
        {
            return c / scale + delta_pos;
        }
        static paths Paths2paths(const ClipperLib::Paths &Ps, double scale, double delta_x = 0.0, double delta_y = 0.0);        // Transform from a Paths variable to a paths variable
        static path Path2path(const ClipperLib::Path &P, double scale, double delta_x = 0.0, double delta_y = 0.0);             // Transform from a Path variable to a path variable
        static ClipperLib::Path path2Path(const path &p, double scale, double delta_x = 0.0, double delta_y = 0.0);             // Transform from a path variable to a Path variable
        static void clearvoid(pathnode *root, const paths &holes, double delta, double area_err, double delta_errscale = 0.02); // Clear all voids by adding extra toolpaths
    private:
        static pathnode *root_offset(const path &contour, const paths &holes, double dis, bool wash = true, double washdis = 0.5, int num_least = 50); // Construct a depth tree of CP toolpaths
        static bool path_subset(const ClipperLib::Path &p1, const ClipperLib::Path &p2);                                                               // Determine whether p1 is enclosed by p2
    public:
        static const int ClipperBound = 1e8;
    };
}