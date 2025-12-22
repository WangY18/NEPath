#pragma once
#include <fstream>
#include <vector>
#include <cmath>
using namespace std;
// Some basic functions are put here.

// The square of 2-norm between points (x1,y1) and (x2,y2)
#define dis_square(x1, y1, x2, y2) (((x1) - (x2)) * ((x1) - (x2)) + ((y1) - (y2)) * ((y1) - (y2)))

// The 2-norm between points (x1,y1) and (x2,y2)
#define dis(x1, y1, x2, y2) (sqrt(((x1) - (x2)) * ((x1) - (x2)) + ((y1) - (y2)) * ((y1) - (y2))))

// The 1-norm between points (x1,y1) and (x2,y2)
#define dis1(x1, y1, x2, y2) ((std::max)(abs((x1) - (x2)), abs((y1) - (y2))))

// The scalar product of vectors (x1,y1) and (x2,y2)
#define innerpro(x1, y1, x2, y2) ((x1) * (x2) + (y1) * (y2))

// The vector product of vectors (x1,y1) and (x2,y2)
#define cropro(x1, y1, x2, y2) ((x1) * (y2) - (x2) * (y1))

// Pi
const double pi = acos(-1.0);

double readcsv_double(ifstream &, bool &);                                                   // read a double variable from an ifstream, which open a .csv file.
double readcsv_double(ifstream &);                                                           // read a double variable from an ifstream, which open a .csv file.
bool subset_cycle_int(int l, int r, int q, bool close_l = false, bool close_r = false);      // For a ring, determine whether q is between l and r.
bool subset_cycle(double l, double r, double q, bool close_l = false, bool close_r = false); // For a ring, determine whether q is between l and r.

bool interset(double xa, double ya, double xc, double yc, double xb, double yb, double xd, double yd);      // Determine whether two segments {(xa,ya),(xb,yb)} and {(xc,yc),(xd,yd)} intersets each other
double intersetion(double xa, double ya, double xc, double yc, double xb, double yb, double xd, double yd); // Find the intersection of two segments {(xa,ya),(xb,yb)} and {(xc,yc),(xd,yd)}
double dropfoot(double x0, double y0, double x1, double y1, double x2, double y2);                          // The foot of the perpendicular drawn from point (x0,y0) to the line cross (x1,y1) and (x2,y2)
double whereneast_point2segment(double x0, double y0, double x1, double y1, double x2, double y2);          // Find the nearest point on segment {(x1,y1),(x2,y2)} of point (x0,y0)
double linear_combine(double x0, double x1, double alpha);                                                  // The linear combination of x0 and x1 with a coefficient alpha