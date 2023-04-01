#include "NEPathPlanner.h"

NEPathPlanner::NEPathPlanner() : delta(1.5), wash(true), washdis(0.5), debug(false), num_least(50) {}

void NEPathPlanner::set_contour(const double* x, const double* y, int length) {
	contour.clear_with_delete();
	if (wash) {
		path p = Curve::wash_dis(x, y, length, washdis);
		contour.steal(p);
	}
	else {
		contour.copy_with_new(x, y, length);
	}
}

void NEPathPlanner::set_contour(const path& contour_new) {
	contour.copy_with_new(contour_new);
}


void NEPathPlanner::addhole(const double* x, const double* y, int length) {
	holes.push_back(path());
	if (wash) {
		path p = Curve::wash_dis(x, y, length, washdis);
		holes[holes.size() - 1].steal(p);
	}
	else {
		holes[holes.size() - 1].copy_with_new(x, y, length);
	}
}

void NEPathPlanner::addhole(const path& hole) {
	holes.push_back(hole);
}

void NEPathPlanner::addholes(const paths& holes_new) {
	for (int i = 0; i < holes_new.size(); ++i) {
		holes.push_back(holes_new[i]);
	}
}

paths NEPathPlanner::Raster(double angle/*=0*/) {
	return DirectionParalle::Raster(contour, holes, delta, angle);
}

paths NEPathPlanner::Raster(const DirectParallelOptions& opts) {
	return DirectionParalle::Raster(contour, holes, opts.delta, opts.angle);
}

paths NEPathPlanner::Zigzag(double angle/*=0*/) {
	return DirectionParalle::Zigzag(contour, holes, delta, angle);
}

paths NEPathPlanner::Zigzag(const DirectParallelOptions& opts) {
	return DirectionParalle::Zigzag(contour, holes, opts.delta, opts.angle);
}

paths NEPathPlanner::CP() {
	return ContourParallel::Contour_Parallel(contour, holes, delta, wash, washdis, num_least);
}

paths NEPathPlanner::CP(const ContourParallelOptions& opts) {
	return ContourParallel::Contour_Parallel(contour, holes, opts.delta, opts.wash, opts.washdis, opts.num_least);
}

paths NEPathPlanner::tool_compensate(double dis) {
	return ContourParallel::tool_compensate(contour, holes, dis, wash, washdis, num_least);
}

paths NEPathPlanner::tool_compensate(const ContourParallelOptions& opts) {
	return ContourParallel::tool_compensate(contour, holes, opts.delta, opts.wash, opts.washdis, opts.num_least);
}