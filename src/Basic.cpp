#include "Basic.h"

namespace nepath
{
    // read a double variable from an ifstream, which open a .csv file.
    // end is the end tag of inFile
    double readcsv_double(std::ifstream &inFile, bool &end)
    {
        int x_int = 0, p = 0;
        double x_frac = 0.0;
        bool minus = false;
        int c = inFile.get();
        if (c == -1)
        {
            end = true;
            return 0.0;
        }
        while ((c < '0' || c > '9') & (c != '-'))
        {
            c = inFile.get();
            if (c == -1)
            {
                end = true;
                return 0.0;
            }
        }
        if (c == '-')
        {
            minus = true;
            c = inFile.get();
        }
        while (c >= '0' && c <= '9')
        {
            x_int = x_int * 10 + c - '0';
            c = inFile.get();
        }
        if (c == '.')
        {
            c = inFile.get();
            double k = 0.1;
            while (c >= '0' && c <= '9')
            {
                x_frac += (c - '0') * k;
                k *= 0.1;
                c = inFile.get();
            }
        }
        if (c == 'e' || c == 'E')
        {
            p = int(readcsv_double(inFile, end));
        }
        double x = minus ? -(x_int + x_frac) : (x_int + x_frac);
        while (p++)
        {
            x *= 0.1;
        }
        end = false;
        return x;
    }

    // read a double variable from an ifstream, which open a .csv file.
    double readcsv_double(std::ifstream &inFile)
    {
        bool flag;
        return readcsv_double(inFile, flag);
    }

    // For a ring, determine whether q is between l and r.
    // For example, 2 is between 1 and 3; 3 is between 2 and 1; 3 is not between 1 and 2.
    // close_l is true means if q==l, return true
    // close_r is true means if q==r, return true
    bool subset_cycle(double l, double r, double q, bool close_l /*=false*/, bool close_r /*=false*/)
    {
        if (!close_l && q == l)
        {
            return false;
        }
        if (!close_r && q == r)
        {
            return false;
        }
        if (close_l && q == l)
        {
            return true;
        }
        if (close_r && q == r)
        {
            return true;
        }
        if (l == r)
        {
            return false;
        }
        else if (l < r)
        {
            return (q > l) && (q < r);
        }
        else
        {
            return (q > l) || (q < r);
        }
    }

    // For a ring, determine whether q is between l and r.
    // For example, 2 is between 1 and 3; 3 is between 2 and 1; 3 is not between 1 and 2.
    // close_l is true means if q==l, return true
    // close_r is true means if q==r, return true
    bool subset_cycle_int(int l, int r, int q, bool close_l /*=false*/, bool close_r /*=false*/)
    {
        if (!close_l && q == l)
        {
            return false;
        }
        if (!close_r && q == r)
        {
            return false;
        }
        if (close_l && q == l)
        {
            return true;
        }
        if (close_r && q == r)
        {
            return true;
        }
        if (l == r)
        {
            return false;
        }
        else if (l < r)
        {
            return (q > l) && (q < r);
        }
        else
        {
            return (q > l) || (q < r);
        }
    }

    // Determine whether two segments {(xa,ya),(xb,yb)} and {(xc,yc),(xd,yd)} intersets each other
    bool interset(double xa, double ya, double xc, double yc, double xb, double yb, double xd, double yd)
    {
        return (cropro(xc - xa, yc - ya, xb - xa, yb - ya) * cropro(xc - xa, yc - ya, xd - xa, yd - ya) < 0) && (cropro(xd - xb, yd - yb, xa - xb, ya - yb) * cropro(xd - xb, yd - yb, xc - xb, yc - yb) < 0);
    }

    // Find the intersection of two segments {(xa,ya),(xb,yb)} and {(xc,yc),(xd,yd)}
    // If the output is t, then the intersection is (xa+t*(xc-xa), ya+t*(yc-ya))
    double intersetion(double xa, double ya, double xc, double yc, double xb, double yb, double xd, double yd)
    {
        return cropro(xb - xa, xb - xd, yb - ya, yb - yd) / cropro(xc - xa, xb - xd, yc - ya, yb - yd);
    }

    // The foot of the perpendicular drawn from point (x0,y0) to the line cross (x1,y1) and (x2,y2)
    // If the output is t, then the foot is (x1+t*(x2-x1), y1+t*(y2-y1))
    double dropfoot(double x0, double y0, double x1, double y1, double x2, double y2)
    {
        return innerpro(x0 - x1, y0 - y1, x2 - x1, y2 - y1) / innerpro(x2 - x1, y2 - y1, x2 - x1, y2 - y1);
    }

    // Find the nearest point on segment {(x1,y1),(x2,y2)} of point (x0,y0)
    // If the output is t, then the nearst point is (x1+t*(x2-x1), y1+t*(y2-y1))
    double whereneast_point2segment(double x0, double y0, double x1, double y1, double x2, double y2)
    {
        double t = dropfoot(x0, y0, x1, y1, x2, y2);
        return std::max(std::min(t, 1.0), 0.0);
    }

    // The linear combination of x0 and x1 with a coefficient alpha
    double linear_combine(double x0, double x1, double alpha)
    {
        return x0 * (1.0 - alpha) + x1 * alpha;
    }
}