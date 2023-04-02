#include "ContourParallel.h"

// Offset a path based on Clipper.
// The path is path(x,y,length), the offsetting distance is dis.
// dis>0 means offsetting the path inside; dis<0 means offsetting the path outside.
// If wash==true, the output paths would be resampled with a uniformly-distributed distance no more than wash_dis, and the number of waypoints are no less than num_least.
paths ContourParallel::OffsetClipper(const double* x, const double* y, double dis, int length, bool wash/*=true*/, double washdis/*=0.5*/, int num_least/*=50*/) {
	double xmax = x[0];
	double xmin = x[0];
	double ymax = y[0];
	double ymin = y[0];
	double delta_x = 0.0;
	double delta_y = 0.0;
	for (register int i = 1; i < length; ++i) {
		xmax = (std::max)(xmax, x[i]);
		xmin = (std::min)(xmin, x[i]);
		ymax = (std::max)(ymax, y[i]);
		ymin = (std::min)(ymin, y[i]);
	}
	delta_x = (xmax + xmin) * 0.5;
	delta_y = (ymax + ymin) * 0.5;
	double scale = ClipperBound / std::max(xmax - xmin, ymax - ymin);

	Path contour;
	for (register int i = 0; i < length; ++i) {
		contour << IntPoint(double2cInt(x[i], scale, delta_x), double2cInt(y[i], scale, delta_y));
	}
	contour << contour[0];
	Paths solution;
	ClipperOffset co;
	co.AddPath(contour, jtRound, etClosedPolygon);
	co.Execute(solution, -dis * scale);

	paths ps;

	if (wash) {
		for (int i = 0; i < solution.size(); ++i) {
			path p0 = Path2path(solution[i], scale, delta_x, delta_y);
			double Length = Curve::TotalLength(p0.x, p0.y, p0.length, true);
			path p = Curve::wash_dis(p0, (std::min)(washdis, Length * 1.0 / num_least));
			ps.push_back(path());
			ps[i].steal(p);
		}
	}
	return ps;
}

// Offset the contour and apply set minus to holes if intersections exist based on Clipper.
// dis>0 means offsetting the path inside; dis<0 means offsetting the path outside.
// If wash==true, the output paths would be resampled with a uniformly-distributed distance no more than wash_dis, and the number of waypoints are no less than num_least.
paths ContourParallel::OffsetClipper(const path& contour, const paths& holes, double dis, bool wash/*=true*/, double washdis/*=0.5*/, int num_least/*=50*/) {
	paths ps_offset = OffsetClipper(contour.x, contour.y, dis, contour.length, wash, washdis);
	paths solution;
	for (int i_offset = 0; i_offset < ps_offset.size(); ++i_offset) {
		paths ps_clip = ContourParallel::cut_holes(ps_offset[i_offset], holes, wash, washdis);
		for (int i = 0; i < ps_clip.size(); ++i) {
			solution.push_back(path());
			solution[solution.size() - 1].steal(ps_clip[i]);
		}
	}
	return solution;
}

// Transform from a double variable to a cInt variable
inline cInt ContourParallel::double2cInt(const double& d, double scale, double delta_pos/*=0.0*/) {
	return (d - delta_pos) * scale;
}

// Transform from a cInt variable to a double variable
inline double ContourParallel::cInt2double(const cInt& c, double scale, double delta_pos/*=0.0*/) {
	return c / scale + delta_pos;
}

// Transform from a Paths variable to a paths variable
paths ContourParallel::Paths2paths(const Paths& Ps, double scale, double delta_x/*=0.0*/, double delta_y/*=0.0*/) {
	paths ps;
	for (int i = 0; i < Ps.size(); ++i) {
		ps.push_back(Path2path(Ps[i], scale, delta_x, delta_y));
	}
	return ps;
}

// Transform from a Path variable to a path variable
path ContourParallel::Path2path(const Path& P, double scale, double delta_x/*=0.0*/, double delta_y/*=0.0*/) {
	path p;
	p.length = P.size();
	p.x = new double[p.length];
	p.y = new double[p.length];
	for (register int i = 0; i < p.length; ++i) {
		p.x[i] = cInt2double(P[i].X, scale, delta_x);
		p.y[i] = cInt2double(P[i].Y, scale, delta_y);
	}
	return p;
}

// Construct a depth tree of CP toolpaths
// The dis is the line width of toolpaths.
// If wash==true, the output toolpaths would be resampled with a uniformly-distributed distance no more than wash_dis, and the number of waypoints are no less than num_least.
pathnode* ContourParallel::root_offset(const path& contour, const paths& holes, double dis, bool wash/*=true*/, double washdis/*=0.5*/, int num_least/*=50*/) {
	pathnode* root = new pathnode(contour);
	root->parent = NULL;
	std::stack<pathnode*> S;
	S.push(root);

	while (!S.empty()) {
		pathnode* pn_parent = S.top();
		S.pop();
		paths ps_offset = OffsetClipper(pn_parent->data.x, pn_parent->data.y, dis, pn_parent->data.length, wash, washdis);
		for (int i_offset = 0; i_offset < ps_offset.size(); ++i_offset) {
			paths ps_clip = ContourParallel::cut_holes(ps_offset[i_offset], holes, wash, washdis);
			for (int i_clip = 0; i_clip < ps_clip.size(); ++i_clip) {
				pathnode* pn_child = new pathnode(ps_clip[i_clip]);
				pn_parent->children.push_back(pn_child);
				pn_child->parent = pn_parent;
				S.push(pn_child);
			}
		}
	}
	return root;
}

// Generate CP toolpath
// contour and holes are the contour and the holes of the slice
// dis>0 is distance between toolpaths
// If wash==true, the output CP toolpaths would be resampled with a uniformly-distributed distance no more than wash_dis, and the number of waypoints are no less than num_least.
// The output is the CP toolpaths
paths ContourParallel::Contour_Parallel(const path& contour, const paths& holes, double dis, bool wash/*=true*/, double washdis/*=0.5*/, int num_least/*=50*/) {
	pathnode* root = root_offset(contour, holes, dis, wash, washdis);
	vector<pathnode*>* dfs = pathnode::DFS_root(root);
	paths solution;
	for (int i = 0; i < dfs->size(); ++i) {
		solution.push_back(path());
		solution[i].length = (*dfs)[i]->data.length;
		solution[i].x = (*dfs)[i]->data.x;
		solution[i].y = (*dfs)[i]->data.y;
		(*dfs)[i]->data.clear_without_delete();
		for (int j = 0; j < (*dfs)[i]->children.size(); ++j) {
			(*dfs)[i]->children[j] = NULL;
		}
	}
	delete dfs;
	delete root;
	return solution;
}

// Apply set minus on contour to holes if intersections exist.
// If wash==true, the output toolpaths would be resampled with a uniformly-distributed distance no more than wash_dis, and the number of waypoints are no less than num_least.
paths ContourParallel::cut_holes(const path& contour, const paths& holes, bool wash/*=true*/, double washdis/*=0.5*/, int num_least/*=50*/) {
	if (!holes.size()) {
		paths ps;
		ps.push_back(contour);
		return ps;
	}

	double xmax = contour.x[0];
	double xmin = contour.x[0];
	double ymax = contour.y[0];
	double ymin = contour.y[0];
	double delta_x = 0.0;
	double delta_y = 0.0;
	for (register int i = 1; i < contour.length; ++i) {
		xmax = (std::max)(xmax, contour.x[i]);
		xmin = (std::min)(xmin, contour.x[i]);
		ymax = (std::max)(ymax, contour.y[i]);
		ymin = (std::min)(ymin, contour.y[i]);
	}
	for (int j = 0; j < holes.size(); ++j) {
		for (register int i = 0; i < holes[j].length; ++i) {
			xmax = (std::max)(xmax, holes[j].x[i]);
			xmin = (std::min)(xmin, holes[j].x[i]);
			ymax = (std::max)(ymax, holes[j].y[i]);
			ymin = (std::min)(ymin, holes[j].y[i]);
		}
	}
	delta_x = (xmax + xmin) * 0.5;
	delta_y = (ymax + ymin) * 0.5;
	double scale = ClipperBound / std::max(xmax - xmin, ymax - ymin);

	Path Contour;
	Paths Holes;
	for (register int i = 0; i < contour.length; ++i) {
		Contour << IntPoint(double2cInt(contour.x[i], scale, delta_x), double2cInt(contour.y[i], scale, delta_y));
	}
	Contour << Contour[0];

	for (int i_hole = 0; i_hole < holes.size(); ++i_hole) {
		Holes.push_back(Path());
		for (register int i = 0; i < holes[i_hole].length; ++i) {
			Holes[Holes.size() - 1] << IntPoint(double2cInt(holes[i_hole].x[i], scale, delta_x), double2cInt(holes[i_hole].y[i], scale, delta_y));
		}
		Holes[Holes.size() - 1] << Holes[Holes.size() - 1][0];
		if (path_subset(Holes[Holes.size() - 1], Contour)) { // Do not apply set minus if the hole is enclosed by the contour
			Holes.pop_back();
		}
	}

	paths solution;
	if (Holes.empty()) { // No intersection
		solution.push_back(contour);
	}
	else {
		Paths Solution;
		Clipper cl;
		cl.AddPath(Contour, ptSubject, true);
		cl.AddPaths(Holes, ptClip, true);
		cl.Execute(ctDifference, Solution);
		for (int i_path = 0; i_path < Solution.size(); ++i_path) {
			if (Solution[i_path].size() < 3) {
				continue;
			}
			solution.push_back(path());
			solution[solution.size() - 1].length = Solution[i_path].size();
			solution[solution.size() - 1].x = new double[solution[solution.size() - 1].length];
			solution[solution.size() - 1].y = new double[solution[solution.size() - 1].length];
			for (register int i = 0; i < Solution[i_path].size(); ++i) {
				solution[solution.size() - 1].x[i] = cInt2double(Solution[i_path][i].X, scale, delta_x);
				solution[solution.size() - 1].y[i] = cInt2double(Solution[i_path][i].Y, scale, delta_y);
			}
		}
	}

	if (wash) {
		for (int i = 0; i < solution.size(); ++i) {
			double Length = Curve::TotalLength(solution[i].x, solution[i].y, solution[i].length, true);
			path p = Curve::wash_dis(solution[i], (std::min)(washdis, Length * 1.0 / num_least));
			solution[i].clear_with_delete();
			solution[i].length = p.length;
			solution[i].x = p.x;
			solution[i].y = p.y;
			p.clear_without_delete();
		}
	}

	return solution;
}

// Apply set minus on contour to holes no matter whether intersections exist.
// dis>0 means offsetting the path inside; dis<0 means offsetting the path outside.
// If wash==true, the output paths would be resampled with a uniformly-distributed distance no more than wash_dis, and the number of waypoints are no less than num_least.
paths ContourParallel::set_minus(const path& contour, const paths& holes, bool onlyouter/*=false*/, bool wash/*=true*/, double washdis/*=0.5*/, int num_least/*=50*/) {
	if (!holes.size()) {
		paths ps;
		ps.push_back(contour);
		return ps;
	}

	double xmax = contour.x[0];
	double xmin = contour.x[0];
	double ymax = contour.y[0];
	double ymin = contour.y[0];
	double delta_x = 0.0;
	double delta_y = 0.0;
	for (register int i = 1; i < contour.length; ++i) {
		xmax = (std::max)(xmax, contour.x[i]);
		xmin = (std::min)(xmin, contour.x[i]);
		ymax = (std::max)(ymax, contour.y[i]);
		ymin = (std::min)(ymin, contour.y[i]);
	}
	for (int j = 0; j < holes.size(); ++j) {
		for (register int i = 0; i < holes[j].length; ++i) {
			xmax = (std::max)(xmax, holes[j].x[i]);
			xmin = (std::min)(xmin, holes[j].x[i]);
			ymax = (std::max)(ymax, holes[j].y[i]);
			ymin = (std::min)(ymin, holes[j].y[i]);
		}
	}
	delta_x = (xmax + xmin) * 0.5;
	delta_y = (ymax + ymin) * 0.5;
	double scale = ClipperBound / std::max(xmax - xmin, ymax - ymin);

	Path Contour;
	Paths Holes;
	for (register int i = 0; i < contour.length; ++i) {
		Contour << IntPoint(double2cInt(contour.x[i], scale, delta_x), double2cInt(contour.y[i], scale, delta_y));
	}
	Contour << Contour[0];

	Clipper cl_hole;
	for (int i_hole = 0; i_hole < holes.size(); ++i_hole) {
		Path Hole;
		for (register int i = 0; i < holes[i_hole].length; ++i) {
			Hole << IntPoint(double2cInt(holes[i_hole].x[i], scale, delta_x), double2cInt(holes[i_hole].y[i], scale, delta_y));
		}
		Hole << Hole[0];
		if (i_hole == 0) {
			cl_hole.AddPath(Hole, ptSubject, true);
		}
		else {
			cl_hole.AddPath(Hole, ptClip, true);
		}
	}
	cl_hole.Execute(ctUnion, Holes);

	Paths IntersectHoles;
	for (int i_hole = 0; i_hole < Holes.size(); ++i_hole) {
		if (!path_subset(Holes[i_hole], Contour)) { // Do not apply set minus if the hole is enclosed by the contour
			IntersectHoles.push_back(Holes[i_hole]);
		}
	}


	paths solution;
	Paths Solution;
	Clipper cl;
	cl.AddPath(Contour, ptSubject, true);
	cl.AddPaths(IntersectHoles, ptClip, true);
	cl.Execute(ctDifference, Solution);
	for (int i_path = 0; i_path < Solution.size(); ++i_path) {
		if (Solution[i_path].size() < 3) {
			continue;
		}
		solution.push_back(path());
		solution[solution.size() - 1].length = Solution[i_path].size();
		solution[solution.size() - 1].x = new double[solution[solution.size() - 1].length];
		solution[solution.size() - 1].y = new double[solution[solution.size() - 1].length];
		for (register int i = 0; i < Solution[i_path].size(); ++i) {
			solution[solution.size() - 1].x[i] = cInt2double(Solution[i_path][i].X, scale, delta_x);
			solution[solution.size() - 1].y[i] = cInt2double(Solution[i_path][i].Y, scale, delta_y);
		}
		if (onlyouter && Curve::AreaCal(solution[solution.size() - 1].x, solution[solution.size() - 1].y, solution[solution.size() - 1].length) <= 0) {
			solution.pop_back();
		}
	}

	if (wash) {
		for (int i = 0; i < solution.size(); ++i) {
			double Length = Curve::TotalLength(solution[i].x, solution[i].y, solution[i].length, true);
			path p = Curve::wash_dis(solution[i], (std::min)(washdis, Length * 1.0 / num_least));
			solution[i].clear_with_delete();
			solution[i].length = p.length;
			solution[i].x = p.x;
			solution[i].y = p.y;
			p.clear_without_delete();
		}
	}

	return solution;

}

// Tool compensate. dis>0 means offsetting the path inside; dis<0 means offsetting the path outside
// dis>0 means offsetting the path inside; dis<0 means offsetting the path outside.
// If wash==true, the output paths would be resampled with a uniformly-distributed distance no more than wash_dis, and the number of waypoints are no less than num_least.
paths ContourParallel::tool_compensate(const path& contour, const paths& holes, double dis, bool wash/*=true*/, double washdis/*=0.5*/, int num_least/*=50*/) {
	double xmax = contour.x[0];
	double xmin = contour.x[0];
	double ymax = contour.y[0];
	double ymin = contour.y[0];
	double delta_x = 0.0;
	double delta_y = 0.0;
	for (register int i = 1; i < contour.length; ++i) {
		xmax = (std::max)(xmax, contour.x[i]);
		xmin = (std::min)(xmin, contour.x[i]);
		ymax = (std::max)(ymax, contour.y[i]);
		ymin = (std::min)(ymin, contour.y[i]);
	}
	for (int j = 0; j < holes.size(); ++j) {
		for (register int i = 0; i < holes[j].length; ++i) {
			xmax = (std::max)(xmax, holes[j].x[i]);
			xmin = (std::min)(xmin, holes[j].x[i]);
			ymax = (std::max)(ymax, holes[j].y[i]);
			ymin = (std::min)(ymin, holes[j].y[i]);
		}
	}
	delta_x = (xmax + xmin) * 0.5;
	delta_y = (ymax + ymin) * 0.5;
	double scale = ClipperBound / std::max(xmax - xmin, ymax - ymin);

	Path Contour;
	for (register int i = 0; i < contour.length; ++i) {
		Contour << IntPoint(double2cInt(contour.x[i], scale, delta_x), double2cInt(contour.y[i], scale, delta_y));
	}
	Contour << Contour[0];

	Paths ContourOffset;
	ClipperOffset co;
	co.AddPath(Contour, jtRound, etClosedPolygon);
	co.Execute(ContourOffset, dis * scale);
	co.Clear();

	Paths HolesOffset;
	for (int i_hole = 0; i_hole < holes.size(); ++i_hole) {
		Path Hole;
		Paths HoleOffset;
		for (register int i = 0; i < holes[i_hole].length; ++i) {
			Hole << IntPoint(double2cInt(holes[i_hole].x[i], scale, delta_x), double2cInt(holes[i_hole].y[i], scale, delta_y));
		}
		Hole << Hole[0];
		co.AddPath(Hole, jtRound, etClosedPolygon);
		co.Execute(HoleOffset, - dis * scale);
		co.Clear();
		for (int i = 0; i < HoleOffset.size(); ++i) {
			HolesOffset.push_back(HoleOffset[i]);
		}
	}

	Paths Solution;
	if (HolesOffset.size()) {
		Clipper cl;
		cl.AddPaths(ContourOffset, ptSubject, true);
		cl.AddPaths(HolesOffset, ptClip, true);
		cl.Execute(ctDifference, Solution);
	}
	else {
		Solution = ContourOffset;
	}

	paths solution = Paths2paths(Solution, scale, delta_x, delta_y);
	if (wash) {
		for (int i = 0; i < Solution.size(); ++i) {
			double Length = Curve::TotalLength(solution[i].x, solution[i].y, solution[i].length, true);
			path p = Curve::wash_dis(solution[i], (std::min)(washdis, Length * 1.0 / num_least));
			solution[i].steal(p);
		}
	}

	return solution;
}

// Determine whether p1 is enclosed by p2
bool ContourParallel::path_subset(const Path& p1, const Path& p2) {
	Paths Solution;
	Clipper cl;
	cl.AddPath(p1, ptSubject, true);
	cl.AddPath(p2, ptClip, true);
	cl.Execute(ctDifference, Solution);
	return !Solution.size();
}

// Transform from a path variable to a Path variable
Path ContourParallel::path2Path(const path& p, double scale, double delta_x/*=0.0*/, double delta_y/*=0.0*/) {
	Path P;
	for (register int i = 0; i < p.length; ++i) {
		P << IntPoint(double2cInt(p.x[i], scale, delta_x), double2cInt(p.y[i], scale, delta_y));
	}
	P << P[0];
	return P;
}

// Clear all voids by adding extra toolpaths
// root is the root node of the depth tree. holes are the holes of the slices. delta is the line width of toolpaths. area_err>0 is the threshold area of underfill
// Toolpaths are added as the leaves of the depth tree
void ContourParallel::clearvoid(pathnode* root, const paths& holes, double delta, double area_err, double delta_errscale/*=0.02*/) {
	paths ps_real_holes = paths();
	for (int i = 0; i < holes.size(); ++i) {
		paths ps = OffsetClipper(holes[i].x, holes[i].y, -delta * 0.5, holes[i].length, true, 0.1, 50);
		for (int j = 0; j < ps.size(); ++j) {
			ps_real_holes.push_back(path());
			ps_real_holes[ps_real_holes.size() - 1].steal(ps[j]);
		}
	}

	stack<pathnode*> S;
	S.push(root);
	while (!S.empty()) {
		pathnode* parent = S.top();
		S.pop();

		if (!parent->children.size() && Curve::AreaCal(parent->data.x, parent->data.y, parent->data.length) <= area_err) {
			paths ps = OffsetClipper(parent->data.x, parent->data.y, -delta * 0.2, parent->data.length, true, 0.1, 50);
			parent->data.clear_with_delete();
			parent->data.steal(ps[0]);
			continue;
		}

		paths ps_holes = ps_real_holes;
		for (int i = 0; i < parent->children.size(); ++i) {
			S.push(parent->children[i]);
			paths ps = OffsetClipper(parent->children[i]->data.x, parent->children[i]->data.y, -delta * (0.5 + delta_errscale), parent->children[i]->data.length, true, 0.1, 50);
			for (int j = 0; j < ps.size(); ++j) {
				ps_holes.push_back(path());
				ps_holes[ps_holes.size() - 1].steal(ps[j]);
			}
		}
		paths ps_parent = OffsetClipper(parent->data.x, parent->data.y, delta * 0.5, parent->data.length, true, 0.1, 50);
		for (int j = 0; j < ps_parent.size(); ++j) {
			paths ps_void = set_minus(ps_parent[j], ps_holes, true, true, 0.1, 50);
			for (int i = 0; i < ps_void.size(); ++i) {
				if (Curve::AreaCal(ps_void[i].x, ps_void[i].y, ps_void[i].length) > area_err) {
					pathnode* childvoid = new pathnode();
					childvoid->parent = parent;
					childvoid->data.steal(ps_void[i]);
					parent->children.push_back(childvoid);
				}
			}
		}
	}
}