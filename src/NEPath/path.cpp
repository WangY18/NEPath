#include <NEPath/path.h>
#include <stack>

namespace nepath
{
    path::path() : x(NULL), y(NULL), length(0) {}

    path::path(const double *x_data, const double *y_data, int length) : x(NULL), y(NULL), length(length)
    {
        if (length)
        {
            x = new double[length];
            y = new double[length];
            for (int i = 0; i < length; ++i)
            {
                this->x[i] = x_data[i];
                this->y[i] = y_data[i];
            }
        }
    }

    path::path(const path &p) : path(p.x, p.y, p.length) {}

    path::~path()
    {
        clear_with_delete();
    }

    // clear all data but do not delete x and y
    void path::clear_without_delete()
    {
        x = NULL;
        y = NULL;
        length = 0;
    }

    // clear all data and delete x and y
    void path::clear_with_delete()
    {
        if (x)
        {
            delete[] x;
        }
        if (y)
        {
            delete[] y;
        }
        x = NULL;
        y = NULL;
        length = 0;
    }

    // copy a path path(x,y,length)
    void path::copy_with_new(const double *xnew, const double *ynew, int length_new)
    {
        length = length_new;
        x = new double[length];
        y = new double[length];
        for (int i = 0; i < length; ++i)
        {
            x[i] = xnew[i];
            y[i] = ynew[i];
        }
    }

    // copy a path pathfrom
    void path::copy_with_new(const path &pathfrom)
    {
        copy_with_new(pathfrom.x, pathfrom.y, pathfrom.length);
    }

    // copy a path p and delete p
    void path::steal(path &p)
    {
        clear_with_delete();
        length = p.length;
        x = p.x;
        y = p.y;
        p.clear_without_delete();
    }

    pathnode::pathnode() : data(path()), parent(NULL) {}

    pathnode::pathnode(const path &p) : data(p), parent(NULL) {}

    pathnode::pathnode(const pathnode &pn) : data(pn.data), parent(pn.parent)
    {
        for (int i = 0; i < pn.children.size(); ++i)
        {
            children.push_back(pn.children[i]);
        }
    }

    // DFS the depth tree and delete the tree
    // root is the root of the depth tree.
    // The output is all paths on the tree in the order of DFS.
    std::vector<pathnode *> *pathnode::DFS_root(pathnode *root)
    {
        std::vector<pathnode *> *dfs = new std::vector<pathnode *>();
        std::stack<pathnode *> S;
        S.push(root);
        while (!S.empty())
        {
            pathnode *path_now = S.top();
            S.pop();
            dfs->push_back(path_now);
            for (int i_child = path_now->children.size() - 1; i_child >= 0; --i_child)
            {
                S.push(path_now->children[i_child]);
            }
        }
        return dfs;
    }

    // find the max x-coordinates of waypoints
    double path::xmax() const
    {
        double xm = x[0];
        for (int i = 1; i < length; ++i)
        {
            xm = (std::max)(xm, x[i]);
        }
        return xm;
    }

    // find the min x-coordinates of waypoints
    double path::xmin() const
    {
        double xm = x[0];
        for (int i = 1; i < length; ++i)
        {
            xm = (std::min)(xm, x[i]);
        }
        return xm;
    }

    // find the max x-coordinates of waypoints
    double path::ymax() const
    {
        double ym = y[0];
        for (int i = 1; i < length; ++i)
        {
            ym = (std::max)(ym, y[i]);
        }
        return ym;
    }

    // find the min y-coordinates of waypoints
    double path::ymin() const
    {
        double ym = y[0];
        for (int i = 1; i < length; ++i)
        {
            ym = (std::min)(ym, y[i]);
        }
        return ym;
    }
}