#pragma once
#include <iostream>
#include <vector>
using namespace std;

struct path
{
	double* x;
	double* y;
	int length;
	path();
	path(const path& p);
	path(const double* x, const double* y, int length);
	~path();
	void clear_without_delete();
	void clear_with_delete();
	void copy_with_new(const path& from);
	void copy_with_new(const double* x, const double* y, int length);
	void steal(path& p);
	double xmax() const;
	double xmin() const;
	double ymax() const;
	double ymin() const;
};

typedef vector<path> paths;

class pathnode {
public:
	pathnode();
	pathnode(const path& p);
	pathnode(const pathnode& pn);
	static vector<pathnode*>* DFS_root(pathnode* root); // 把树转化为向量并清除树
public:
	path data;
	vector<pathnode*> children;
	pathnode* parent;
};