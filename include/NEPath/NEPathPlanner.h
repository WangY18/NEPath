#pragma once
#include <NEPath/setup_NEPath.h>
#include <NEPath/Curve.h>
#include <NEPath/ContourParallel.h>
#include <NEPath/PlanningOptions.h>
#include <NEPath/DirectionParallel.h>
#include <NEPath/NonEquidistant.h>
// NEPathPlanner is a class to plan toolpaths, connect toolpaths, and perform tool compensating.

namespace nepath
{
    class NEPathPlanner
    {
    public:
        void set_contour(const double *x, const double *y, int length, bool wash = true, double washdis = 0.2, int num_least = 50); // Set the contour (outer boundary) of the slice.
        void set_contour(const path &contour_new, bool wash = true, double washdis = 0.2, int num_least = 50);                      // Set the contour (outer boundary) of the slice.
        void addhole(const path &hole_new, bool wash = true, double washdis = 0.2, int num_least = 50);                             // Add a new hole (inner boundary) onto the slice.
        void addhole(const double *x, const double *y, int length, bool wash = true, double washdis = 0.2, int num_least = 50);     // Add a new hole (inner boundary) onto the slice.
        void addholes(const paths &holes_new, bool wash = true, double washdis = 0.2, int num_least = 50);                          // Add some new holes (inner boundaries) onto the slice.
        paths tool_compensate(const ContourParallelOptions &opts);                                                                  // Offset the contour and holes of the slice with a distance
    public:
        paths IQOP(const NonEquidistantOptions &opts, bool log = true); // Generate IQOP toolpath
        paths Raster(const DirectParallelOptions &opts);                // Generate Raster toolpath
        paths Zigzag(const DirectParallelOptions &opts);                // Generate Zigzag toolpath
        paths CP(const ContourParallelOptions &opts);                   // Generate CP toolpath
    public:
    public:
        path contour;
        paths holes;
    };
}