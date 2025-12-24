#pragma once
#include <iostream>
#include <fstream>
#include <stack>
#include <algorithm>
#include <NEPath/Basic.h>
#include <NEPath/path.h>
// UnderFillSolution is a struct to store information of underfill.
// SharpTurnSolution is a struct to store information of sharp corners.
// Curve is a class to perform basic 2D geometric operations.
// Sharp corner is determined by: Helmut Pottmann, Johannes Wallner, Qi-Xing Huang, and Yong-Liang Yang.Integral invariants for robust geometry processing. Computer Aided Geometric Design, 26(1): 37-60, 2009.

namespace nepath
{
    struct UnderFillSolution
    {
        bool **map_slice = NULL;     // map_slice[i][j]==true iff point (xs[i],ys[j]) is in the slice
        bool **map_delta = NULL;     // map_delta[i][j]==true iff point (xs[i],ys[j]) is fill by the given toolpaths
        int nx = -1;                 // length of xs
        int ny = -1;                 // length of ys
        double *xs = NULL;           // Discrete points on x-axis
        double *ys = NULL;           // Discrete points on y-axis
        double underfillrate = -1.0; // underfill rate
        void clear();                // initial
    };

    struct SharpTurnSolution
    {
        int length = 0;             // the number of waypoints (the same as toolpaths)
        double radius = -1.0;       // radius of the circle
        double *AreaPercent = NULL; // Percent of area on one side of the toolpath. If the toolpath is not closed, AreaPercent[0] and AreaPercent[length-1] is set as -1.0
        double threshold = -1.0;    // Threshold of area to determine sharp corners.
        bool *SharpTurn = NULL;     // SharpTurn[i]==true iff the i-th waypoints is a sharp corner.If the toolpath is not closed, SharpTurn[0] and SharpTurn[end] is set as false
        bool close = false;         // close==true iff the toolpath is closed
        void clear();               // initial
    };

    class Curve
    {
    public:                                                                                                                                                             // Basic 2D geometric operations
        static double interp_id(const double *x, int length, double id);                                                                                                // interp of the index id in x
        static double nearest_id(const double *x, const double *y, int length, double x0, double y0);                                                                   // the index of nearest point on path(x,y,length) to point (x0,y0)
        static double distance_point2path(const double *x, const double *y, int length, double x0, double y0);                                                          // the distance between path(x,y,length) and point (x0,y0)
        static double furthest_id(const double *x, const double *y, int length, double x0, double y0);                                                                  // the index of furthest point on path(x,y,length) to point (x0,y0)
        static double curves_nearest(const path &p, const path &q, double &id1, double &id2);                                                                           // the index of nearest points between path p and q
        static path *rotate(const path &p, double angle);                                                                                                               // rotate the path p with angle (rad) anticlockwise
        static paths *rotate(const paths &ps, double angle);                                                                                                            // rotate the paths ps with angle (rad) anticlockwise
        static bool interset_path(const path &a, const path &b);                                                                                                        // determine whether path a and b interset each other
        static bool interset_path_id(const path &a, const path &b, double &ida, double &idb);                                                                           // find the point where path a and b interset each other
        static void Ndir(const double *x, const double *y, int length, double *&nx, double *&ny);                                                                       // normal directions of path(x,y,length)
        static double AreaCal(const double *x, const double *y, int length);                                                                                            // area enclosed by path(x,y,length)
        static void DiffLength(double *&dl, const double *x, const double *y, int length, bool poly = true);                                                            // distance between waypoints of path(x,y,length)
        static double *DiffLength(const double *x, const double *y, int length, bool poly = true);                                                                      // distance between waypoints of path(x,y,length)
        static double TotalLength(const double *x, const double *y, int length, bool poly = true);                                                                      // length of path(x,y,length)
        static double LengthBetween(const double *x, const double *y, int length, int from, int to);                                                                    // length along path(x,y,length) between point (x[from],y[from]) to point (x[to],y[to])
        static double LengthBetween(const double *x, const double *y, int length, double from, double to);                                                              // length along path(x,y,length) between point (x[from],y[from]) to point (x[to],y[to])
        static void OffsetNaive(const double *x, const double *y, const double *delta, int length, double *&xnew, double *&ynew, double *nx = NULL, double *ny = NULL); // offset path(x,y,length) on direction (nx,ny) with a const distance delta
        static void OffsetNaive(const double *x, const double *y, double delta, int length, double *&xnew, double *&ynew, double *nx = NULL, double *ny = NULL);        // offset path(x,y,length) on direction (nx,ny) with variable distances delta
    public:                                                                                                                                                             // Underfill and sharp corner
        static UnderFillSolution UnderFill(const path &contour, const paths &holes, const paths &ps, double delta, double reratio);                                     // calculate the underfill
        static double UnderFillRate(const path &contour, const paths &holes, const paths &ps, double delta, double reratio);                                            // calculate the underfill rate
        static SharpTurnSolution SharpTurn_Invariant(const path &p, double radius, double threshold = 0.3, bool close = false, double washdis = -1.0);                  // calculate the sharp corners
        static int SharpTurnNum_Invariant(const path &p, double radius, double threshold = 0.3, bool close = false, double washdis = -1.0);                             // calculate the number of sharp corners
    private:
        static double AreaInvariant_OnePoint(const path &p, double radius, int id, bool close); // calculate the area invariant of a waypoint
    private:
        struct point
        {
            double x;
            double y;
            point() : x(0), y(0) {}
            point(double X, double Y) : x(X), y(Y) {}
            point(const point &p) : x(p.x), y(p.y) {}
        };
        static bool cmp_Raster(const point &a, const point &b);

    public:                                                                                         // CFS
        static double BackDis(const double *x, const double *y, int length, double id, double dis); // the point on path(x,y,length) back (x[id],y[id]) with a length of dis along the path
        static double ForDis(const double *x, const double *y, int length, double id, double dis);  // the point on path(x,y,length) forward (x[id],y[id]) with a length of dis along the path
    public:
        static path wash_dis(const path &p, double dis, int num_least = -1);                                                          // resample the path p with a distance approximately equal to dis
        static path wash_dis(const double *x, const double *y, int length, double dis, int num_least = -1, bool output_poly = false); // resample the path p with a distance approximately equal to dis
    };
}