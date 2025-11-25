#include "Curve.h"

// find the point where path a and b interset each other
// If a and b interset each other, then return true.
// ida is the index of intersection on a, while idb is the index of intersection on b.
bool Curve::interset_path_id(const path& a, const path& b, double& ida, double& idb) {
	for (int i = 0; i < a.length; ++i) {
		for (int j = 0; j < b.length; ++j) {
			if (interset(a.x[i], a.y[i],
				a.x[(i + 1) % a.length], a.y[(i + 1) % a.length],
				b.x[j], b.y[j],
				b.x[(j + 1) % b.length], b.y[(j + 1) % b.length])) {
				double t = intersetion(a.x[i], a.y[i],
					a.x[(i + 1) % a.length], a.y[(i + 1) % a.length],
					b.x[j], b.y[j],
					b.x[(j + 1) % b.length], b.y[(j + 1) % b.length]);
				ida = i + t;
				idb = j + t;
				return true;
			}
		}
	}
	ida = idb = -1.0;
	return false;
}

// determine whether path a and b interset each other
bool Curve::interset_path(const path& a, const path& b) {
	double t1, t2;
	return interset_path_id(a, b, t1, t2);
}

// rotate the path p with angle (rad) anticlockwise
path* Curve::rotate(const path& p, double angle) {
	path* pr = new path();
	pr->length = p.length;
	pr->x = new double[pr->length];
	pr->y = new double[pr->length];
	for (int i = 0; i < pr->length; ++i) {
		pr->x[i] = p.x[i] * cos(angle) - p.y[i] * sin(angle);
		pr->y[i] = p.x[i] * sin(angle) + p.y[i] * cos(angle);
	}
	return pr;
}

// rotate the paths ps with angle (rad) anticlockwise
paths* Curve::rotate(const paths& ps, double angle) {
	paths* psr = new paths();
	path* hr = NULL;
	for (int i = 0; i < ps.size(); ++i) {
		psr->push_back(path());
		hr = Curve::rotate(ps[i], angle);
		(*psr)[i].length = hr->length;
		(*psr)[i].x = hr->x;
		(*psr)[i].y = hr->y;
		hr->clear_without_delete();
	}
	delete hr;
	return psr;
}

// normal directions of path(x,y,length)
// the output is store in nx and ny. (nx[i],ny[i]) is the normal direction at point (x[i],y[i])
void Curve::Ndir(const double* x, const double* y, int length, double*& nx, double*& ny) {
	if (!nx) {
		nx = new double[length];
	}
	if (!ny) {
		ny = new double[length];
	}
	for (int i = 0; i < length; ++i) {
		double dx = x[(i + 1) % length] - x[(i + length - 1) % length];
		double dy = y[(i + 1) % length] - y[(i + length - 1) % length];
		double dl = sqrt(dx * dx + dy * dy);
		nx[i] = -dy / dl;
		ny[i] = dx / dl;
	}
}

// area enclosed by path(x,y,length)
double Curve::AreaCal(const double* x, const double* y, int length) {
	double A2 = 0.0;
	for (int i = 0; i < length; ++i) {
		A2 += x[i] * y[(i + 1) % length] - y[i] * x[(i + 1) % length];
	}
	return 0.5 * A2;
}

// distance between waypoints of path(x,y,length)
// dl[i] is the distance between point (x[i],y[i]) and (x[i+1],y[i+1])
// poly==true iff the path is closed
// the output is stored in dl
void Curve::DiffLength(double*& dl, const double* x, const double* y, int length, bool poly/*=true*/) {
	if (!dl) {
		dl = new double[length + poly - 1];
	}
	for (int i = 0; i < length - 1; ++i) {
		dl[i] = sqrt((x[i + 1] - x[i]) * (x[i + 1] - x[i]) + (y[i + 1] - y[i]) * (y[i + 1] - y[i]));
	}
	if (poly) {
		dl[length - 1] = sqrt((x[length - 1] - x[0]) * (x[length - 1] - x[0]) + (y[length - 1] - y[0]) * (y[length - 1] - y[0]));
	}
}

// distance between waypoints of path(x,y,length)
// dl[i] is the distance between point (x[i],y[i]) and (x[i+1],y[i+1])
// poly==true iff the path is closed
double* Curve::DiffLength(const double* x, const double* y, int length, bool poly/*=true*/) {
	double* dl;
	DiffLength(dl, x, y, length, poly);
	return dl;
}

// length of path(x,y,length)
// poly==true iff the path is closed
double Curve::TotalLength(const double* x, const double* y, int length, bool poly/*=true*/) {
	double L = 0.0;
	for (int i = 0; i < length - 1; ++i) {
		L += sqrt((x[i + 1] - x[i]) * (x[i + 1] - x[i]) + (y[i + 1] - y[i]) * (y[i + 1] - y[i]));
	}
	if (poly) {
		L += sqrt((x[length - 1] - x[0]) * (x[length - 1] - x[0]) + (y[length - 1] - y[0]) * (y[length - 1] - y[0]));
	}
	return L;
}

// length along path(x,y,length) between point (x[from],y[from]) to point (x[to],y[to])
double Curve::LengthBetween(const double* x, const double* y, int length, int from, int to) {
	if (from == to) {
		return 0;
	}
	double L = 0.0;
	for (int i = from; i != to; i = (i + 1) % length) {
		L += sqrt((x[(i + 1) % length] - x[i]) * (x[(i + 1) % length] - x[i]) + (y[(i + 1) % length] - y[i]) * (y[(i + 1) % length] - y[i]));
	}
	return L;
}

// length along path(x,y,length) between point (x[from],y[from]) to point (x[to],y[to])
double Curve::LengthBetween(const double* x, const double* y, int length, double from, double to) {
	int id_floor_from = floor(from);
	int id_floor_to = floor(to);
	if ((id_floor_from == id_floor_to) && (to > from)) {
		return (to - from) * dis(x[id_floor_from], y[id_floor_from], x[(id_floor_from + 1) % length], y[(id_floor_from + 1) % length]);
	}
	double L = (1.0 - from + id_floor_from) * dis(x[id_floor_from], y[id_floor_from], x[(id_floor_from + 1) % length], y[(id_floor_from + 1) % length]);
	L += (to - id_floor_to) * dis(x[id_floor_to], y[id_floor_to], x[(id_floor_to + 1) % length], y[(id_floor_to + 1) % length]);
	return L + LengthBetween(x, y, length, (id_floor_from + 1) % length, id_floor_to);
}

// offset path(x,y,length) on direction (nx,ny) with a const distance delta
// By default, nx and ny are NULL. They will be calculated as the normal directions.
// The output is stored in xnew and ynew
void Curve::OffsetNaive(const double* x, const double* y, const double* delta, int length, double*& xnew, double*& ynew, double* nx/*=NULL*/, double* ny/*=NULL*/) {
	if (!(nx && ny)) {
		Ndir(x, y, length, nx, ny);
	}
	xnew = new double[length];
	ynew = new double[length];
	for (int i = 0; i < length; ++i) {
		xnew[i] = x[i] + delta[i] * nx[i];
		ynew[i] = y[i] + delta[i] * ny[i];
	}
}

// offset path(x,y,length) on direction (nx,ny) with variable distances delta
// By default, nx and ny are NULL. They will be calculated as the normal directions.
// The output is stored in xnew and ynew
void Curve::OffsetNaive(const double* x, const double* y, double delta, int length, double*& xnew, double*& ynew, double* nx/*=NULL*/, double* ny/*=NULL*/) {
	if (!(nx && ny)) {
		Ndir(x, y, length, nx, ny);
	}
	if (!xnew) {
		xnew = new double[length];
	}
	if (!ynew) {
		ynew = new double[length];
	}
	for (int i = 0; i < length; ++i) {
		xnew[i] = x[i] + delta * nx[i];
		ynew[i] = y[i] + delta * ny[i];
	}
}

// resample the path p with a distance approximately equal to dis
path Curve::wash_dis(const path& p, double dis) {
	return wash_dis(p.x, p.y, p.length, dis);
}

// resample the path p(x,y,length) with a distance approximately equal to dis
path Curve::wash_dis(const double* x, const double* y, int length, double dis, bool output_poly/*=false*/) {
	double* dl = new double[length];
	double sumL = 0;
	for (int i = 0; i < length; ++i) {
		dl[i] = dis(x[i], y[i], x[(i + 1) % length], y[(i + 1) % length]);
		sumL += dl[i];
	}
	double Lnow = 0;
	path pnew;
	pnew.length = ceil(sumL / dis);
	pnew.x = new double[pnew.length + output_poly];
	pnew.y = new double[pnew.length + output_poly];
	pnew.x[0] = x[0];
	pnew.y[0] = y[0];
	int j = 1;
	for (int i = 0; i < length;) {
		if (Lnow + dl[i] < sumL * j / pnew.length) {
			Lnow += dl[i];
			++i;
		}
		else {
			double alpha = (sumL * j / pnew.length - Lnow) / dl[i];
			pnew.x[j] = x[i] * (1 - alpha) + x[(i + 1) % length] * alpha;
			pnew.y[j] = y[i] * (1 - alpha) + y[(i + 1) % length] * alpha;
			++j;
			if (j >= pnew.length) {
				break;
			}
		}
	}
	if (output_poly) {
		++pnew.length;
		pnew.x[pnew.length - 1] = pnew.x[0];
		pnew.y[pnew.length - 1] = pnew.y[0];
	}
	delete[] dl;
	return pnew;
}

// the point on path(x,y,length) back (x[id],y[id]) with a length of dis along the path
double Curve::BackDis(const double* x, const double* y, int length, double id, double dis) {
	if (dis < 0) {
		return ForDis(x, y, length, id, -dis);
	}
	double xid = interp_id(x, length, id);
	double yid = interp_id(y, length, id);
	int id_floor = floor(id);
	double sumL = dis(xid, yid, x[id_floor], y[id_floor]);
	if (sumL >= dis) {
		double alpha = dis / sumL;
		return id_floor * alpha + id * (1.0 - alpha);
	}
	for (int i = (id_floor + length - 1) % length; ; ) {
		double dl = dis(x[i], y[i], x[(i + 1) % length], y[(i + 1) % length]);
		if (sumL + dl <= dis) {
			sumL += dl;
			i = (i + length - 1) % length;
		}
		else {
			double alpha = (dis - sumL) / dl; // alphaԽС��Խ����i+1
			return i + 1.0 - alpha; // i * alpha + (i + 1) * (1.0 - alpha)
		}
	}
}

// the point on path(x,y,length) forward (x[id],y[id]) with a length of dis along the path
double Curve::ForDis(const double* x, const double* y, int length, double id, double dis) {
	if (dis < 0) {
		return BackDis(x, y, length, id, -dis);
	}
	double xid = interp_id(x, length, id);
	double yid = interp_id(y, length, id);
	int id_ceil = int(ceil(id)) % length;
	double sumL = dis(xid, yid, x[id_ceil], y[id_ceil]);
	if (sumL >= dis) {
		double alpha = dis / sumL;
		return int(ceil(id)) * alpha + id * (1.0 - alpha);
	}
	for (int i = id_ceil; ; ) {
		double dl = dis(x[i], y[i], x[(i + 1) % length], y[(i + 1) % length]);
		if (sumL + dl <= dis) {
			sumL += dl;
			i = (i + 1) % length;
		}
		else {
			double alpha = (dis - sumL) / dl; // alphaԽС��Խ����i+1
			return i + alpha; // i * (1.0 - alpha) + (i + 1) * alpha
		}
	}
}

// interp of the index id in x
double Curve::interp_id(const double* x, int length, double id) {
	int id_floor = floor(id);
	double alpha = id - id_floor;
	return x[id_floor] * (1.0 - alpha) + x[(id_floor + 1) % length] * alpha;
}

// the distance between path(x,y,length) and point (x0,y0) 
double Curve::distance_point2path(const double* x, const double* y, int length, double x0, double y0) {
	double id = nearest_id(x, y, length, x0, y0);
	double xm = interp_id(x, length, id);
	double ym = interp_id(y, length, id);
	return dis(x0, y0, xm, ym);
}

//the index of nearest point on path(x,y,length) to point (x0,y0)
double Curve::nearest_id(const double* x, const double* y, int length, double x0, double y0) {
	double idm = 0.0;
	double dism = dis(x[0], y[0], x0, y0);
	for (int i = 0; i < length; ++i) {
		double alpha = whereneast_point2segment(x0, y0, x[i], y[i], x[(i + 1) % length], y[(i + 1) % length]);
		double xnow = linear_combine(x[i], x[(i + 1) % length], alpha);
		double ynow = linear_combine(y[i], y[(i + 1) % length], alpha);
		double disnow = dis(xnow, ynow, x0, y0);
		if (disnow < dism) {
			dism = disnow;
			idm = i + alpha;
		}
	}
	return idm;
}

// the index of furthest point on path(x,y,length) to point (x0,y0) 
double Curve::furthest_id(const double* x, const double* y, int length, double x0, double y0) {
	int idm = 0;
	double dism = dis(x[0], y[0], x0, y0);
	for (int i = 1; i < length; ++i) {
		double disnow = dis(x[i], y[i], x0, y0);
		if (disnow < dism) {
			dism = disnow;
			idm = i;
		}
	}
	return idm;
}

// the index of nearest points between path p and q
// idp is the index of nearest point on p, while idq is the index of nearest point on q
double Curve::curves_nearest(const path& p, const path& q, double& idp, double& idq) {
	if (interset_path_id(p, q, idp, idq)) {
		return 0;
	}

	int id = 0;
	double id_ = 0;
	double dism1 = dis(p.x[0], p.y[0], q.x[0], q.y[0]);
	for (int i = 0; i < p.length; ++i) {
		double id2 = nearest_id(q.x, q.y, q.length, p.x[i], p.y[i]);
		double disnow = dis(p.x[i], p.y[i],
			interp_id(q.x, q.length, id2),
			interp_id(q.y, q.length, id2));
		if (disnow < dism1) {
			id = i;
			id_ = id2;
			dism1 = disnow;
		}
	}
	idp = id;
	idq = id_;

	id = 0;
	id_ = 0;
	double dism2 = dis(p.x[0], p.y[0], q.x[0], q.y[0]);
	for (int i = 0; i < q.length; ++i) {
		double id2 = nearest_id(p.x, p.y, p.length, q.x[i], q.y[i]);
		double disnow = dis(q.x[i], q.y[i],
			interp_id(p.x, p.length, id2),
			interp_id(p.y, p.length, id2));
		if (disnow < dism2) {
			id = i;
			id_ = id2;
			dism2 = disnow;
		}
	}
	if (dism2 < dism1) {
		idp = id_;
		idq = id;
		return dism2;
	}
	return dism1;
}

// calculate the underfill
// ps is toolpaths; delta is linewidth; reratio is the reratio, i.e., distance between pixel points
UnderFillSolution Curve::UnderFill(const path& contour, const paths& holes, const paths& ps, double delta, double reratio) {
	UnderFillSolution sol;

	// Bounding box
	double xmax = contour.xmax();
	double xmin = contour.xmin();
	double ymax = contour.ymax();
	double ymin = contour.ymin();
	for (int i = 0; i < holes.size(); ++i) {
		xmax = (std::max)(xmax, holes[i].xmax());
		xmin = (std::min)(xmin, holes[i].xmin());
		ymax = (std::max)(ymax, holes[i].ymax());
		ymin = (std::min)(ymin, holes[i].ymin());
	}
	for (int i = 0; i < ps.size(); ++i) {
		xmax = (std::max)(xmax, ps[i].xmax());
		xmin = (std::min)(xmin, ps[i].xmin());
		ymax = (std::max)(ymax, ps[i].ymax());
		ymin = (std::min)(ymin, ps[i].ymin());
	}
	xmax += delta;
	ymax += delta;
	xmin -= delta;
	ymin -= delta;
	
	// Pixelization
	sol.nx = ceil((xmax - xmin) / reratio) + 1;
	sol.ny = ceil((ymax - ymin) / reratio) + 1;
	sol.xs = new double[sol.nx]();
	for (int i = 0; i < sol.nx; ++i) {
		sol.xs[i] = (xmin + xmax - (sol.nx - 1) * reratio) * 0.5 + i * reratio;
	}
	sol.ys = new double[sol.ny]();
	for (int i = 0; i < sol.ny; ++i) {
		sol.ys[i] = (ymin + ymax - (sol.ny - 1) * reratio) * 0.5 + i * reratio;
	}

	// Determine which points are in the slice
	sol.map_slice = new bool* [sol.nx];
	for (int i = 0; i < sol.nx; ++i) {
		sol.map_slice[i] = new bool[sol.ny]();
		for (int j = 0; j < sol.ny; ++j) {
			sol.map_slice[i][j] = false;
		}
	}
	const path** all_contours = new const path * [holes.size() + 1];
	for (int i = 0; i < holes.size(); ++i) {
		all_contours[i] = holes.data() + i;
	}
	all_contours[holes.size()] = &contour;
	vector<point> intersection;
	for (int i = 0; i <= holes.size(); ++i) {
		for (int j = 0; j < all_contours[i]->length; ++j) { // ��x�и���߶ΰ�����㡢�������յ�
			if (all_contours[i]->x[j] == all_contours[i]->x[(j + 1) % all_contours[i]->length]) {
				continue;
			}
			else if (all_contours[i]->x[j] < all_contours[i]->x[(j + 1) % all_contours[i]->length]) {
				for (int k = ceil((all_contours[i]->x[j] - sol.xs[0]) / reratio) - 1; k < sol.nx && sol.xs[k] < all_contours[i]->x[(j + 1) % all_contours[i]->length]; ++k) {
					if (k < 0 || sol.xs[k] < all_contours[i]->x[j]) { // ��ֹ��ֵ�����ְ�����
						continue;
					}
					double alpha = (sol.xs[k] - all_contours[i]->x[j]) / (all_contours[i]->x[(j + 1) % all_contours[i]->length] - all_contours[i]->x[j]);
					intersection.push_back(point(sol.xs[k],
						(1.0 - alpha) * all_contours[i]->y[j] + alpha * all_contours[i]->y[(j + 1) % all_contours[i]->length]));
				}
			}
			else {
				for (int k = floor((all_contours[i]->x[j] - sol.xs[0]) / reratio); k >= 0 && sol.xs[k] > all_contours[i]->x[(j + 1) % all_contours[i]->length]; --k) {
					double alpha = (sol.xs[k] - all_contours[i]->x[j]) / (all_contours[i]->x[(j + 1) % all_contours[i]->length] - all_contours[i]->x[j]);
					intersection.push_back(point(sol.xs[k],
						(1.0 - alpha) * all_contours[i]->y[j] + alpha * all_contours[i]->y[(j + 1) % all_contours[i]->length]));
				}
			}
		}
	}
	sort(intersection.data(), intersection.data() + intersection.size(), cmp_Raster);
	for (int k = 0; k < intersection.size() - 1; k += 2) {
		int i = round((intersection[k].x - sol.xs[0]) / reratio);
		if (i != round((intersection[k + 1].x - sol.xs[0]) / reratio)) {
			--k;
			continue;
		}
		for (int j = ceil((intersection[k].y - sol.ys[0]) / reratio) - 1; j < sol.ny && sol.ys[j] <= intersection[k + 1].y; ++j) {
			if (j < 0 || sol.ys[j] < intersection[k].y) {
				continue;
			}
			sol.map_slice[i][j] = true;
		}
	}

	// Determine which points have distances shorter than 0.5*delta to toolpaths
	sol.map_delta = new bool* [sol.nx];
	for (int i = 0; i < sol.nx; ++i) {
		sol.map_delta[i] = new bool[sol.ny]();
		for (int j = 0; j < sol.ny; ++j) {
			sol.map_delta[i][j] = false;
		}
	}
	for (int ip = 0; ip < ps.size(); ++ip) {
		for (int is = 0; is < ps[ip].length - 1; ++is) {
			double x1 = ps[ip].x[is];
			double y1 = ps[ip].y[is];
			double x2 = ps[ip].x[is + 1];
			double y2 = ps[ip].y[is + 1];
			for (int i = (std::max)(0, (int)floor(((std::min)(x1, x2) - delta * 0.5 - sol.xs[0]) / reratio)); i < sol.nx && sol.xs[i] <= (std::max)(x1, x2) + 0.5 * delta; ++i) {
				for (int j = (std::max)(0, (int)floor(((std::min)(y1, y2) - delta * 0.5 - sol.ys[0]) / reratio)); j < sol.ny && sol.ys[j] <= (std::max)(y1, y2) + 0.5 * delta; ++j) {
					if (((cropro(sol.xs[i] - x1, sol.ys[j] - y1, x2 - x1, y2 - y1)) * (cropro(sol.xs[i] - x1, sol.ys[j] - y1, x2 - x1, y2 - y1)) <= 0.25 * delta * delta * ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))) && 
						(((sol.xs[i] - x1) * (x2 - x1) + (sol.ys[j] - y1) * (y2 - y1)) * ((sol.xs[i] - x2) * (x2 - x1) + (sol.ys[j] - y2) * (y2 - y1)) < 0)) {
						sol.map_delta[i][j] = true;
					}
				}
			}
		}
		for (int is = 0; is < ps[ip].length; ++is) {
			double x = ps[ip].x[is];
			double y = ps[ip].y[is];
			for (int i = (std::max)(0, (int)floor((x - delta * 0.5 - sol.xs[0]) / reratio)); i < sol.nx && sol.xs[i] <= x + 0.5 * delta; ++i) {
				for (int j = (std::max)(0, (int)floor((y - delta * 0.5 - sol.ys[0]) / reratio)); j < sol.ny && sol.ys[j] <= y + 0.5 * delta; ++j) {
					if ((sol.xs[i] - x) * (sol.xs[i] - x) + (sol.ys[j] - y) * (sol.ys[j] - y) <= 0.25 * delta * delta) {
						sol.map_delta[i][j] = true;
					}
				}
			}
		}
	}

	int num_slice = 0;
	int num_slicedelta = 0;
	for (int i = 0; i < sol.nx; ++i) {
		for (int j = 0; j < sol.ny; ++j) {
			num_slice += sol.map_slice[i][j];
			num_slicedelta += (sol.map_slice[i][j] & sol.map_delta[i][j]);
		}
	}
	sol.underfillrate = 1.0 - 1.0 * num_slicedelta / num_slice;

	return sol;
}

// calculate the underfill rate
// ps is toolpaths; delta is linewidth; reratio is the reratio, i.e., distance between pixel points
double Curve::UnderFillRate(const path& contour, const paths& holes, const paths& ps, double delta, double reratio) {
	UnderFillSolution sol = UnderFill(contour, holes, ps, delta, reratio);
	double rate = sol.underfillrate;
	sol.clear();
	return rate;
}

bool Curve::cmp_Raster(const point& a, const point& b) {
	if (abs(a.x - b.x) < 1e-8) {
		return a.y < b.y;
	}
	return a.x < b.x;
}

// initial
void UnderFillSolution::clear() {
	if (map_slice) {
		for (int i = 0; i < nx; ++i) {
			delete[] map_slice[i];
		}
		delete[] map_slice;
		map_slice = NULL;
	}
	if (map_delta) {
		for (int i = 0; i < nx; ++i) {
			delete[] map_delta[i];
		}
		delete[] map_delta;
		map_delta = NULL;
	}
	nx = 0;
	ny = 0;
	underfillrate = -1.0;
}

// calculate the area invariant of a waypoint
// If fail, return -pi*radius^2
// Pottmann H, Wallner J, Huang Q X, et al. Integral invariants for robust geometry processing[J]. Computer Aided Geometric Design, 2009, 26(1): 37-60.
double Curve::AreaInvariant_OnePoint(const path& p, double radius, int id, bool close) {
	double xminus = 0.0;
	double yminus = 0.0;
	double xplus = 0.0;
	double yplus = 0.0;
	double xstar = p.x[id];
	double ystar = p.y[id];
	double idminus = -1;
	double idplus = -1;

	// �����ҵ�����
	if (!close) { // ���պ�
		for (int i = id + 1; i < p.length; ++i) { // ����plus
			if (dis(xstar, ystar, p.x[i], p.y[i]) >= radius) {
				double h = abs(cropro(xstar - p.x[i - 1], ystar - p.y[i - 1], xstar - p.x[i], ystar - p.y[i])) / dis(p.x[i], p.y[i], p.x[i - 1], p.y[i - 1]); // �����εĸ�
				double lstar = sqrt(radius * radius - h * h);
				double l1 = sqrt(dis_square(xstar, ystar, p.x[i - 1], p.y[i - 1]) - h * h); // < lstar
				double l2 = sqrt(dis_square(xstar, ystar, p.x[i], p.y[i]) - h * h); // > lstar
				double alpha = (lstar - l1) / (l2 - l1);
				idplus = i - 1.0 + alpha;
				xplus = (1.0 - alpha) * p.x[i - 1] + alpha * p.x[i];
				yplus = (1.0 - alpha) * p.y[i - 1] + alpha * p.y[i];
				break;
			}
		}
		if (idplus < 0) { // δ�ཻ
			return -pi * radius * radius;
		}
		for (int i = id - 1; i >= 0; --i) { // ����minus
			if (dis(xstar, ystar, p.x[i], p.y[i]) >= radius) {
				double h = abs(cropro(xstar - p.x[i + 1], ystar - p.y[i + 1], xstar - p.x[i], ystar - p.y[i])) / dis(p.x[i], p.y[i], p.x[i + 1], p.y[i + 1]); // �����εĸ�
				double lstar = sqrt(radius * radius - h * h);
				double l1 = sqrt(dis_square(xstar, ystar, p.x[i + 1], p.y[i + 1]) - h * h); // < lstar
				double l2 = sqrt(dis_square(xstar, ystar, p.x[i], p.y[i]) - h * h); // > lstar
				double alpha = (lstar - l1) / (l2 - l1);
				idminus = i + 1.0 - alpha;
				xminus = (1.0 - alpha) * p.x[i + 1] + alpha * p.x[i];
				yminus = (1.0 - alpha) * p.y[i + 1] + alpha * p.y[i];
				break;
			}
		}
		if (idminus < 0) { // δ�ཻ
			return -pi * radius * radius;
		}
	}
	else { // �պ�
		for (int i = (id + 1) % p.length; subset_cycle((id + 1) % p.length, id, i, true, false); i = (i + 1) % p.length) { // ����plus
			if (dis(xstar, ystar, p.x[i], p.y[i]) >= radius) {
				int j = (i + p.length - 1) % p.length;
				double h = abs(cropro(xstar - p.x[j], ystar - p.y[j], xstar - p.x[i], ystar - p.y[i])) / dis(p.x[i], p.y[i], p.x[j], p.y[j]); // �����εĸ�
				double lstar = sqrt(radius * radius - h * h);
				double l1 = sqrt(dis_square(xstar, ystar, p.x[j], p.y[j]) - h * h); // < lstar
				double l2 = sqrt(dis_square(xstar, ystar, p.x[i], p.y[i]) - h * h); // > lstar
				double alpha = (lstar - l1) / (l2 - l1);
				idplus = i - 1.0 + alpha;
				if (idplus < 0) {
					idplus += p.length;
				}
				xplus = (1.0 - alpha) * p.x[j] + alpha * p.x[i];
				yplus = (1.0 - alpha) * p.y[j] + alpha * p.y[i];
				break;
			}
		}
		if (idplus < 0) { // δ�ཻ
			return -pi * radius * radius;
		}
		for (int i = (id + p.length - 1) % p.length; subset_cycle(id, (id + p.length - 1) % p.length, i, false, true); i = (i + p.length - 1) % p.length) { // ����minus
			if (dis(xstar, ystar, p.x[i], p.y[i]) >= radius) {
				int j = (i + 1) % p.length;
				double h = abs(cropro(xstar - p.x[j], ystar - p.y[j], xstar - p.x[i], ystar - p.y[i])) / dis(p.x[i], p.y[i], p.x[j], p.y[j]); // �����εĸ�
				double lstar = sqrt(radius * radius - h * h);
				double l1 = sqrt(dis_square(xstar, ystar, p.x[j], p.y[j]) - h * h); // < lstar
				double l2 = sqrt(dis_square(xstar, ystar, p.x[i], p.y[i]) - h * h); // > lstar
				double alpha = (lstar - l1) / (l2 - l1);
				idminus = i + 1.0 - alpha;
				if (idminus > p.length) {
					idminus -= p.length;
				}
				xminus = (1.0 - alpha) * p.x[j] + alpha * p.x[i];
				yminus = (1.0 - alpha) * p.y[j] + alpha * p.y[i];
				break;
			}
		}
		if (idminus < 0) { // δ�ཻ
			return -pi * radius * radius;
		}
	}
	
	// idplus, idminus������������м����ټ��һ��id
	// �����A1
	double Area1 = 0.0;
	for (int i = (int)ceil(idminus) % p.length; subset_cycle_int((int)ceil(idminus) % p.length, (int)floor(idplus), i, true, false); i = (i + 1) % p.length) {
		Area1 += cropro(p.x[i], p.y[i], p.x[(i + 1) % p.length], p.y[(i + 1) % p.length]);
	}
	Area1 += cropro(xminus, yminus, p.x[(int)ceil(idminus) % p.length], p.y[(int)ceil(idminus) % p.length]);
	Area1 += cropro(p.x[(int)floor(idplus)], p.y[(int)floor(idplus)], xplus, yplus);
	Area1 += cropro(xplus, yplus, xstar, ystar);
	Area1 += cropro(xstar, ystar, xminus, yminus);
	Area1 *= 0.5;

	// �����A2
	double thetaplus = atan2(yplus - ystar, xplus - xstar);
	double thetaminus = atan2(yminus - ystar, xminus - xstar);
	if (thetaminus < thetaplus) {
		thetaminus += 2 * pi;
	}
	double Area2 = 0.5 * (thetaminus - thetaplus) * radius * radius;

	return Area1 + Area2;
}

// calculate the sharp corners
// ps is toolpaths; radius is radius of the circle; threshold of area to determine sharp corners
// If more than threshold area is on the same side of toolpaths, then the waypoint is determined as a sharp corner
// close==true iff p is closed and the length of output is the number of waypoints plus 1
// washdis<=0 iff wash is not conducted. washdis>0 iff wash is conducted with a wash dis as washdis
// Pottmann H, Wallner J, Huang Q X, et al. Integral invariants for robust geometry processing[J]. Computer Aided Geometric Design, 2009, 26(1): 37-60.
SharpTurnSolution Curve::SharpTurn_Invariant(const path& p, double radius, double threshold/*=0.3*/, bool close/*=false*/, double washdis/*=-1.0*/) {
	SharpTurnSolution sol;
	sol.length = p.length;
	sol.radius = radius;
	sol.threshold = threshold;
	if (sol.threshold < 0.5) {
		sol.threshold = 1.0 - sol.threshold;
	}
	sol.AreaPercent = new double[sol.length];
	sol.SharpTurn = new bool[sol.length];
	sol.close = close;
	for (int i = 0; i < p.length; ++i) {
		if (i == 0 && !close) {
			sol.AreaPercent[i] = -1.0;
			sol.SharpTurn[i] = false;
		}
		else if (i == p.length - 1 && !close) {
			sol.AreaPercent[i] = -1.0;
			sol.SharpTurn[i] = false;
		}
		else {
			double Area = AreaInvariant_OnePoint(p, sol.radius, i, close);
			if (Area > 0) {
				sol.AreaPercent[i] = Area / (pi * radius * radius);
				if (sol.AreaPercent[i] < 0.5) {
					sol.AreaPercent[i] = 1.0 - sol.AreaPercent[i];
				}
				sol.SharpTurn[i] = (sol.AreaPercent[i] >= sol.threshold);
			}
			else {
				sol.AreaPercent[i] = -1.0;
				sol.SharpTurn[i] = false;
			}
		}
	}

	if (washdis > 0) {
		if (!sol.close) {
			for (int i = 0; i < sol.length; ++i) {
				if (sol.SharpTurn[i]) {
					int j = i;
					for (int k = i + 1; k < sol.length; ++k) {
						if (dis(p.x[i], p.y[i], p.x[k], p.y[k]) <= washdis) {
							if (sol.SharpTurn[k]) {
								j = k;
							}
						}
						else {
							break;
						}
					}
					if (j > i) {
						for (int k = i; k < j; ++k) {
							sol.SharpTurn[k] = false;
						}
						for (int k = j + 1; k < sol.length; ++k) {
							if (dis(p.x[j], p.y[j], p.x[k], p.y[k]) <= washdis) {
								sol.SharpTurn[k] = false;
							}
							else {
								break;
							}
						}
						i = j;
					}
				}
			}
		}
		else {
			bool flag_first = true;
			for (int i = 0; i < sol.length; ++i) {
				if (sol.SharpTurn[i]) {
					if (flag_first) {
						flag_first = false;
						for (int j = (i + 1) % sol.length; j != i; j = (j + 1) % sol.length) {
							if (dis(p.x[i], p.y[i], p.x[j], p.y[j]) <= washdis) {
								sol.SharpTurn[j] = false;
							}
							else {
								break;
							}
						}
						for (int j = (i + sol.length - 1) % sol.length; j != i; j = (j + sol.length - 1) % sol.length) {
							if (dis(p.x[i], p.y[i], p.x[j], p.y[j]) <= washdis) {
								sol.SharpTurn[j] = false;
							}
							else {
								break;
							}
						}
					}
					else {
						int j = i;
						for (int k = (i + 1) % sol.length; k != i; k = (k + 1) % sol.length) {
							if (dis(p.x[i], p.y[i], p.x[k], p.y[k]) <= washdis) {
								if (sol.SharpTurn[k]) {
									j = k;
								}
							}
							else {
								break;
							}
						}
						if (j != i) {
							for (int k = i; k != j; k = (k + 1) % sol.length) {
								sol.SharpTurn[k] = false;
							}
							for (int k = (j + 1) % sol.length; k != j; k = (k + 1) % sol.length) {
								if (dis(p.x[j], p.y[j], p.x[k], p.y[k]) <= washdis) {
									sol.SharpTurn[k] = false;
								}
								else {
									break;
								}
							}
							i = j;
						}
					}
				}
			}
		}
	}

	return sol;
}

// calculate the number of sharp corners
// ps is toolpaths; radius is radius of the circle; threshold of area to determine sharp corners
// If more than threshold area is on the same side of toolpaths, then the waypoint is determined as a sharp corner
// close==true iff p is closed and the length of output is the number of waypoints plus 1
// washdis<=0 iff wash is not conducted. washdis>0 iff wash is conducted with a wash dis as washdis
int Curve::SharpTurnNum_Invariant(const path& p, double radius, double threshold/*=0.3*/, bool close/*=false*/, double washdis/*=-1.0*/) {
	SharpTurnSolution sol = SharpTurn_Invariant(p, radius, threshold, close, washdis);
	int num = 0;
	for (int i = 0; i < sol.length; ++i) {
		num += sol.SharpTurn[i];
	}
	sol.clear();
	return num;
}

// initial
void SharpTurnSolution::clear() {
	delete[] AreaPercent;
	AreaPercent = NULL;
	delete[] SharpTurn;
	SharpTurn = NULL;
}