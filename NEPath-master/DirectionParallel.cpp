#include "DirectionParallel.h"
#include "Basic.h"
#include "ContourParallel.h"

// 光栅算法
paths DirectionParalle::Raster(const path& contour, const paths& holes, double dis, double angle/*=0*/) {
	if (!contour.length) {
		return paths();
	}

	path* contour_rotate = Curve::rotate(contour, -angle);
	paths* holes_rotate = Curve::rotate(holes, -angle);

	double xmax = contour_rotate->x[0];
	double xmin = contour_rotate->x[0];
	double ymax = contour_rotate->y[0];
	double ymin = contour_rotate->y[0];
	for (register int i = 1; i < contour_rotate->length; ++i) {
		xmax = (std::max)(xmax, contour_rotate->x[i]);
		xmin = (std::min)(xmin, contour_rotate->x[i]);
		ymax = (std::max)(ymax, contour_rotate->y[i]);
		ymin = (std::min)(ymin, contour_rotate->y[i]);
	}
	double delta_x = (xmax + xmin) * 0.5;
	double delta_y = (ymax + ymin) * 0.5;
	double scale = ContourParallel::ClipperBound / std::max(xmax - xmin, ymax - ymin);

	Path Contour;
	Paths Holes;
	for (register int i = 0; i < contour_rotate->length; ++i) {
		Contour << IntPoint(ContourParallel::double2cInt(contour_rotate->x[i], scale, delta_x), ContourParallel::double2cInt(contour_rotate->y[i], scale, delta_y));
	}
	Contour << Contour[0];

	for (int i_hole = 0; i_hole < holes_rotate->size(); ++i_hole) {
		Holes.push_back(Path());
		for (register int i = 0; i < (*holes_rotate)[i_hole].length; ++i) {
			Holes[Holes.size() - 1] << IntPoint(ContourParallel::double2cInt((*holes_rotate)[i_hole].x[i], scale, delta_x), ContourParallel::double2cInt((*holes_rotate)[i_hole].y[i], scale, delta_y));
		}
		Holes[Holes.size() - 1] << Holes[Holes.size() - 1][0];
	}

	Paths Solution = Raster(Contour, Holes, dis, scale);
	paths solution = ContourParallel::Paths2paths(Solution, scale, delta_x, delta_y);

	delete contour_rotate;
	delete holes_rotate;

	return *(Curve::rotate(solution, angle));
}

Paths DirectionParalle::Raster(const Path& contour, const Paths& holes, double dis, double scale) {
	Paths ps;
	Paths contour_now; // 实际用于计算的轮廓

	contour_now.push_back(contour);
	for (register int i = 0; i < holes.size(); ++i) {
		contour_now.push_back(holes[i]);
	}

	// 接下来用水平线切割
	cInt ymax = contour_now[0][0].Y, ymin = contour_now[0][0].Y;
	for (register int i = 0; i < contour_now.size(); ++i) {
		for (register int j = 0; j < contour_now[i].size(); ++j) {
			ymax = max(ymax, contour_now[i][j].Y);
			ymin = min(ymin, contour_now[i][j].Y);
		}
	}
	int num = ceil((ymax - ymin) / (scale * dis)) - 1;
	cInt Dis = (ymax - ymin) / (num + 1);
	cInt y0 = ymin + Dis / 2;

	vector<IntPoint> intersection; // 交点集
	for (register int i = 0; i < contour_now.size(); ++i) {
		for (register int j = 0; j < contour_now[i].size(); ++j) {
			int I = ceil(1.0 * (contour_now[i][j].Y - y0) / Dis);
			int II = ceil(1.0 * (contour_now[i][(j + 1) % contour_now[i].size()].Y - y0) / Dis);
			for (register int k = min(I, II); k < max(I, II); ++k) {
				double lambda = 1.0 * (y0 + k * Dis - contour_now[i][j].Y) / (contour_now[i][(j + 1) % contour_now[i].size()].Y - contour_now[i][j].Y);
				intersection.push_back(IntPoint(cInt(contour_now[i][j].X * (1.0 - lambda) + contour_now[i][(j + 1) % contour_now[i].size()].X * lambda), y0 + k * Dis));
			}
		}
	}

	// 生成线段集
	sort(intersection.data(), intersection.data() + intersection.size(), cmp_Raster);
	for (register int i = 0; i < intersection.size(); i += 2) {
		Path p;
		p.push_back(intersection[i]);
		p.push_back(intersection[i + 1]);
		ps.push_back(p);
	}

	return ps;
}


bool DirectionParalle::cmp_Raster(const IntPoint& a, const IntPoint& b) {
	if (a.Y == b.Y) {
		return a.X < b.X;
	}
	return a.Y < b.Y;
}

Paths DirectionParalle::Zigzag(const Path& contour, const Paths& holes, double dis, double scale) {
	Paths ps;
	Paths contour_now; // 实际用于计算的轮廓

	// 轮廓平行几圈，使用刀补算法
	contour_now.push_back(contour);
	for (register int i = 0; i < holes.size(); ++i) {
		contour_now.push_back(holes[i]);
	}

	// 接下来用水平线切割
	cInt ymax = contour_now[0][0].Y, ymin = contour_now[0][0].Y;
	for (register int i = 0; i < contour_now.size(); ++i) {
		for (register int j = 0; j < contour_now[i].size(); ++j) {
			ymax = max(ymax, contour_now[i][j].Y);
			ymin = min(ymin, contour_now[i][j].Y);
		}
	}
	int num = ceil((ymax - ymin) / (scale * dis)) - 1;
	cInt Dis = (ymax - ymin) / (num + 1);
	cInt y0 = ymin + Dis / 2;


	// 定义交点类
	struct InterPoint {
		IntPoint point;
		unsigned int idpath;
		unsigned int idpoint;
		InterPoint(cInt x, cInt y, unsigned int path, unsigned int point) :
			point(IntPoint(x, y)), idpath(path), idpoint(point) {}
		bool operator < (const InterPoint& p) {
			return DirectionParalle::cmp_Raster(this->point, p.point);
		}
		static bool nextto(const InterPoint& a, const InterPoint& b, int size) {
			// 同一条path上且相邻
			return (a.idpath == b.idpath) && ((a.idpoint == (b.idpoint + 1) % size) || (b.idpoint == (a.idpoint + 1) % size));
		}
	};
	vector<InterPoint> intersection; // 交点集
	int* size_pathinter = new int[contour_now.size()](); // 每条path上交点个数
	for (register int i = 0; i < contour_now.size(); ++i) {
		unsigned int idpoint = 0;
		for (register int j = 0; j < contour_now[i].size(); ++j) {
			int I = ceil(1.0 * (contour_now[i][j].Y - y0) / Dis);
			int II = ceil(1.0 * (contour_now[i][(j + 1) % contour_now[i].size()].Y - y0) / Dis);
			for (register int k = min(I, II); k < max(I, II); ++k) {
				double lambda = 1.0 * (y0 + k * Dis - contour_now[i][j].Y) / (contour_now[i][(j + 1) % contour_now[i].size()].Y - contour_now[i][j].Y);
				intersection.push_back(InterPoint(cInt(contour_now[i][j].X * (1.0 - lambda) + contour_now[i][(j + 1) % contour_now[i].size()].X * lambda), y0 + k * Dis, i, idpoint++));
			}
		}
		size_pathinter[i] = idpoint;
	}

	// 生成线段集
	sort(intersection.data(), intersection.data() + intersection.size());
	vector<vector<InterPoint>> paths;
	for (register int i = 0; i < intersection.size(); i += 2) {
		bool flag = false;
		for (register int j = paths.size() - 1; j >= 0; --j) {
			if (InterPoint::nextto(intersection[i], paths[j][paths[j].size() - 1], size_pathinter[intersection[i].idpath])) {
				paths[j].push_back(intersection[i]);
				paths[j].push_back(intersection[i + 1]);
				flag = true;
				break;
			}
			if (InterPoint::nextto(intersection[i + 1], paths[j][paths[j].size() - 1], size_pathinter[intersection[i + 1].idpath])) {
				paths[j].push_back(intersection[i + 1]);
				paths[j].push_back(intersection[i]);
				flag = true;
				break;
			}
		}
		if (!flag) { // 开一个新的
			vector<InterPoint> path_now;
			path_now.push_back(intersection[i]);
			path_now.push_back(intersection[i + 1]);
			paths.push_back(path_now);
		}
	}
	for (register int i = 0; i < paths.size(); ++i) {
		Path p;
		for (register int j = 0; j < paths[i].size(); ++j) {
			p.push_back(paths[i][j].point);
		}
		ps.push_back(p);
	}

	return ps;
}

// Zigzag算法
paths DirectionParalle::Zigzag(const path& contour, const paths& holes, double dis, double angle/*=0*/) {
	if (!contour.length) {
		return paths();
	}

	path* contour_rotate = Curve::rotate(contour, -angle);
	paths* holes_rotate = Curve::rotate(holes, -angle);

	double xmax = contour_rotate->x[0];
	double xmin = contour_rotate->x[0];
	double ymax = contour_rotate->y[0];
	double ymin = contour_rotate->y[0];
	for (register int i = 1; i < contour_rotate->length; ++i) {
		xmax = (std::max)(xmax, contour_rotate->x[i]);
		xmin = (std::min)(xmin, contour_rotate->x[i]);
		ymax = (std::max)(ymax, contour_rotate->y[i]);
		ymin = (std::min)(ymin, contour_rotate->y[i]);
	}
	double delta_x = (xmax + xmin) * 0.5;
	double delta_y = (ymax + ymin) * 0.5;
	double scale = ContourParallel::ClipperBound / std::max(xmax - xmin, ymax - ymin);

	Path Contour;
	Paths Holes;
	for (register int i = 0; i < contour_rotate->length; ++i) {
		Contour << IntPoint(ContourParallel::double2cInt(contour_rotate->x[i], scale, delta_x), ContourParallel::double2cInt(contour_rotate->y[i], scale, delta_y));
	}
	Contour << Contour[0];

	for (int i_hole = 0; i_hole < holes_rotate->size(); ++i_hole) {
		Holes.push_back(Path());
		for (register int i = 0; i < (*holes_rotate)[i_hole].length; ++i) {
			Holes[Holes.size() - 1] << IntPoint(ContourParallel::double2cInt((*holes_rotate)[i_hole].x[i], scale, delta_x), ContourParallel::double2cInt((*holes_rotate)[i_hole].y[i], scale, delta_y));
		}
		Holes[Holes.size() - 1] << Holes[Holes.size() - 1][0];
	}

	Paths Solution = Zigzag(Contour, Holes, dis, scale);
	paths solution = ContourParallel::Paths2paths(Solution, scale, delta_x, delta_y);

	delete contour_rotate;
	delete holes_rotate;

	return *(Curve::rotate(solution, angle));
}