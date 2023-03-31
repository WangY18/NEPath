#include "Basic.h"

double readcsv_double(ifstream& inFile, bool& end) {
	int x_int = 0, p = 0;
	double x_frac = 0.0;
	bool minus = false;
	int c = inFile.get();
	if (c == -1) {
		end = true;
		return 0.0;
	}
	while ((c < '0' || c>'9') & (c != '-')) {
		c = inFile.get();
		if (c == -1) {
			end = true;
			return 0.0;
		}
	}
	if (c == '-') {
		minus = true;
		c = inFile.get();
	}
	while (c >= '0' && c <= '9') {
		x_int = x_int * 10 + c - '0';
		c = inFile.get();
	}
	if (c == '.') {
		c = inFile.get();
		double k = 0.1;
		while (c >= '0' && c <= '9') {
			x_frac += (c - '0') * k;
			k *= 0.1;
			c = inFile.get();
		}
	}
	if (c == 'e' || c == 'E') {
		p = int(readcsv_double(inFile, end));
	}
	double x = minus ? -(x_int + x_frac) : (x_int + x_frac);
	while (p++) {
		x *= 0.1;
	}
	end = false;
	return x;
}

double readcsv_double(ifstream& inFile) {
	bool flag;
	return readcsv_double(inFile, flag);
}

int readcsv_xy(char const* filename, vector<double>& x, vector<double>& y) {
	if (x.size() || y.size()) {
		printf("Warning: the vector will be clear when read *.csv.\n");
	}
	ifstream inFile(filename, ios::in);
	if (!inFile) {
		printf("File %s does not exist.\n", filename);
		return 0;
	}
	while (!inFile.eof()) {
		bool end = false;
		double t = readcsv_double(inFile, end);
		if (!end) {
			x.push_back(t);
			y.push_back(readcsv_double(inFile));
		}
	}
	return x.size();
}

// 判断环内包含关系
//template<typename T1, typename T2, typename T3>
//bool subset_cycle(T1 l, T2 r, T3 q, bool close_l/*=false*/, bool close_r/*=false*/) {
bool subset_cycle(double l, double r, double q, bool close_l/*=false*/, bool close_r/*=false*/) {
	if (!close_l && q == l) {
		return false;
	}
	if (!close_r && q == r) {
		return false;
	}
	if (close_l && q == l) {
		return true;
	}
	if (close_r && q == r) {
		return true;
	}
	if (l == r) {
		return false;
	}
	else if (l < r) {
		return (q > l) && (q < r);
	}
	else {
		return (q > l) || (q < r);
	}
}

// 判断环内包含关系
bool subset_cycle_int(int l, int r, int q, bool close_l/*=false*/, bool close_r/*=false*/) {
	if (!close_l && q == l) {
		return false;
	}
	if (!close_r && q == r) {
		return false;
	}
	if (close_l && q == l) {
		return true;
	}
	if (close_r && q == r) {
		return true;
	}
	if (l == r) {
		return false;
	}
	else if (l < r) {
		return (q > l) && (q < r);
	}
	else {
		return (q > l) || (q < r);
	}
}

// 判断线段相交
// TODO：重合情形
bool interset(double xa, double ya, double xc, double yc, double xb, double yb, double xd, double yd) {
	return (cropro(xc - xa, yc - ya, xb - xa, yb - ya) * cropro(xc - xa, yc - ya, xd - xa, yd - ya) < 0) && (cropro(xd - xb, yd - yb, xa - xb, ya - yb) * cropro(xd - xb, yd - yb, xc - xb, yc - yb) < 0);
}

// 判断直线交点
// 返回值：t，交点为(xa+t*(xc-xa), ya+t*(yc-ya))
double intersetion(double xa, double ya, double xc, double yc, double xb, double yb, double xd, double yd) {
	return cropro(xb - xa, xb - xd, yb - ya, yb - yd) / cropro(xc - xa, xb - xd, yc - ya, yb - yd);
}

// 垂足
double dropfoot(double x0, double y0, double x1, double y1, double x2, double y2) {
	return innerpro(x0 - x1, y0 - y1, x2 - x1, y2 - y1) / innerpro(x2 - x1, y2 - y1, x2 - x1, y2 - y1);
}

// 点到线段最近的位置
double whereneast_point2segment(double x0, double y0, double x1, double y1, double x2, double y2) {
	double t = dropfoot(x0, y0, x1, y1, x2, y2);
	return max(min(t, 1.0), 0.0);
}

/*
template<typename T>
inline T linear_combine(T x0, T x1, double alpha) {
	return x0 * (1.0 - alpha) + x1 * alpha;
}*/

double linear_combine(double x0, double x1, double alpha) {
	return x0 * (1.0 - alpha) + x1 * alpha;
}