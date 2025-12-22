#pragma once
#include <iostream>
#include <vector>
using namespace std;
// path is a struct to store information of toolpaths.
// paths is a vector of path.
// pathnode is a class to store information of nodes in a depth tree.

struct path
{
    double *x;  // x-coordinate of waypoints
    double *y;  // y-coordinate of waypoints
    int length; // the number of waypoints
    path();
    path(const path &p);
    path(const double *x, const double *y, int length);
    ~path();
    void clear_without_delete();                                      // clear all data but do not delete x and y
    void clear_with_delete();                                         // clear all data and delete x and y
    void copy_with_new(const path &pathfrom);                         // copy a path pathfrom
    void copy_with_new(const double *x, const double *y, int length); // copy a path path(x,y,length)
    void steal(path &p);                                              // copy a path p and delete p
    double xmax() const;                                              // find the max x-coordinates of waypoints
    double xmin() const;                                              // find the min x-coordinates of waypoints
    double ymax() const;                                              // find the max y-coordinates of waypoints
    double ymin() const;                                              // find the min y-coordinates of waypoints
};

typedef vector<path> paths;

class pathnode
{
public:
    pathnode();
    pathnode(const path &p);
    pathnode(const pathnode &pn);
    static vector<pathnode *> *DFS_root(pathnode *root); // DFS the depth tree and delete the tree
public:
    path data;                   // the path of this node
    vector<pathnode *> children; // all children of this node
    pathnode *parent;            // the parent of this node (NULL means it is the root)
};