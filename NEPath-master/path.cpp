#pragma once
#include "path.h"
#include <stack>


path::path() : x(NULL), y(NULL), length(NULL) {}

path::path(const double* x_data, const double* y_data, int length) : x(NULL), y(NULL), length(length) {
	if (length) {
		x = new double[length];
		y = new double[length];
		for (int i = 0; i < length; ++i) {
			this->x[i] = x_data[i];
			this->y[i] = y_data[i];
		}
	}
}

path::path(const path& p) : path(p.x, p.y, p.length) {}

path::~path() {
	clear_with_delete();
}

void path::clear_without_delete() {
	x = NULL;
	y = NULL;
	length = 0;
}

void path::clear_with_delete() {
	if (x) {
		delete[] x;
	}
	if (y) {
		delete[] y;
	}
	x = NULL;
	y = NULL;
	length = 0;
}

void path::copy_with_new(const double* xnew, const double* ynew, int length_new) {
	length = length_new;
	x = new double[length];
	y = new double[length];
	for (int i = 0; i < length; ++i) {
		x[i] = xnew[i];
		y[i] = ynew[i];
	}
}

void path::copy_with_new(const path& from) {
	copy_with_new(from.x, from.y, from.length);
}

void path::steal(path& p) {
	clear_with_delete();
	length = p.length;
	x = p.x;
	y = p.y;
	p.clear_without_delete();
}

pathnode::pathnode() : data(path()), parent(NULL) {}

pathnode::pathnode(const path& p) : data(p), parent(NULL) {}

pathnode::pathnode(const pathnode& pn) : data(pn.data), parent(pn.parent) {
	for (int i = 0; i < pn.children.size(); ++i) {
		children.push_back(pn.children[i]);
	}
}

// 把树转化为向量，并且清除root
vector<pathnode*>* pathnode::DFS_root(pathnode* root) {
	vector<pathnode*>* dfs = new vector<pathnode*>();
	stack<pathnode*> S;
	S.push(root);
	while (!S.empty()) {
		pathnode* path_now = S.top();
		S.pop();
		dfs->push_back(path_now);
		for (int i_child = path_now->children.size() - 1; i_child >= 0; --i_child) {
			S.push(path_now->children[i_child]);
			//path_now->children[i_child] = NULL;
			//path_now->children.pop_back();
		}
	}
	return dfs;
}

double path::xmax() const {
	double xm = x[0];
	for (int i = 1; i < length; ++i) {
		xm = (std::max)(xm, x[i]);
	}
	return xm;
}

double path::xmin() const {
	double xm = x[0];
	for (int i = 1; i < length; ++i) {
		xm = (std::min)(xm, x[i]);
	}
	return xm;
}

double path::ymax() const {
	double ym = y[0];
	for (int i = 1; i < length; ++i) {
		ym = (std::max)(ym, y[i]);
	}
	return ym;
}

double path::ymin() const {
	double ym = y[0];
	for (int i = 1; i < length; ++i) {
		ym = (std::min)(ym, y[i]);
	}
	return ym;
}