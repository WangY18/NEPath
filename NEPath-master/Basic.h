#pragma once
#include <fstream>
#include <vector>
#include <cmath>
using namespace std;

#define dis_square(x1,y1,x2,y2) (((x1)-(x2))*((x1)-(x2))+((y1)-(y2))*((y1)-(y2)))
#define dis(x1,y1,x2,y2) (sqrt(((x1)-(x2))*((x1)-(x2))+((y1)-(y2))*((y1)-(y2))))
#define dis1(x1,y1,x2,y2) ((std::max)(abs((x1)-(x2)),abs((y1)-(y2))))
#define innerpro(x1,y1,x2,y2) ((x1)*(x2)+(y1)*(y2))
#define cropro(x1,y1,x2,y2) ((x1)*(y2)-(x2)*(y1))
const double pi = acos(-1.0);

double readcsv_double(ifstream&, bool&);
double readcsv_double(ifstream&);
int readcsv_xy(char const*, vector<double>&, vector<double>&);

bool subset_cycle_int(int l, int r, int q, bool close_l = false, bool close_r = false);
bool subset_cycle(double l, double r, double q, bool close_l = false, bool close_r = false);

bool interset(double xa, double ya, double xc, double yc, double xb, double yb, double xd, double yd);
double intersetion(double xa, double ya, double xc, double yc, double xb, double yb, double xd, double yd);
double dropfoot(double x0, double y0, double x1, double y1, double x2, double y2); // 垂足
double whereneast_point2segment(double x0, double y0, double x1, double y1, double x2, double y2); // 点到线段最近的位置
double linear_combine(double x0, double x1, double alpha);