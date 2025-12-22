#include "DirectionParallel.h"
#include "Basic.h"
#include "ContourParallel.h"

namespace nepath
{
    // Generate Raster toolpaths
    // dis is the distance between toolpaths; angle (rad) is the angle between toolpaths and the x-axis.
    paths DirectionParalle::Raster(const path &contour, const paths &holes, double dis, double angle /*=0*/)
    {
        if (!contour.length)
        {
            return paths();
        }

        // Rotate the boundaries
        path *contour_rotate = Curve::rotate(contour, -angle);
        paths *holes_rotate = Curve::rotate(holes, -angle);

        // Transform the rotated boundaries into ClipperLib::Path form
        double xmax = contour_rotate->x[0];
        double xmin = contour_rotate->x[0];
        double ymax = contour_rotate->y[0];
        double ymin = contour_rotate->y[0];
        for (int i = 1; i < contour_rotate->length; ++i)
        {
            xmax = (std::max)(xmax, contour_rotate->x[i]);
            xmin = (std::min)(xmin, contour_rotate->x[i]);
            ymax = (std::max)(ymax, contour_rotate->y[i]);
            ymin = (std::min)(ymin, contour_rotate->y[i]);
        }
        double delta_x = (xmax + xmin) * 0.5;
        double delta_y = (ymax + ymin) * 0.5;
        double scale = ContourParallel::ClipperBound / std::max(xmax - xmin, ymax - ymin);

        ClipperLib::Path Contour;
        ClipperLib::Paths Holes;
        for (int i = 0; i < contour_rotate->length; ++i)
        {
            Contour << ClipperLib::IntPoint(ContourParallel::double2cInt(contour_rotate->x[i], scale, delta_x), ContourParallel::double2cInt(contour_rotate->y[i], scale, delta_y));
        }
        Contour << Contour[0];

        for (int i_hole = 0; i_hole < holes_rotate->size(); ++i_hole)
        {
            Holes.push_back(ClipperLib::Path());
            for (int i = 0; i < (*holes_rotate)[i_hole].length; ++i)
            {
                Holes[Holes.size() - 1] << ClipperLib::IntPoint(ContourParallel::double2cInt((*holes_rotate)[i_hole].x[i], scale, delta_x), ContourParallel::double2cInt((*holes_rotate)[i_hole].y[i], scale, delta_y));
            }
            Holes[Holes.size() - 1] << Holes[Holes.size() - 1][0];
        }

        // Generate Raster toolpaths
        ClipperLib::Paths Solution = Raster(Contour, Holes, dis, scale);

        // Transform the Raster toolpaths into path form
        paths solution = ContourParallel::Paths2paths(Solution, scale, delta_x, delta_y);

        delete contour_rotate;
        delete holes_rotate;

        return *(Curve::rotate(solution, angle));
    }

    // Generate Raster toolpaths
    // dis is the distance between toolpaths; angle (rad) is the angle between toolpaths and the x-axis.
    ClipperLib::Paths DirectionParalle::Raster(const ClipperLib::Path &contour, const ClipperLib::Paths &holes, double dis, double scale)
    {
        ClipperLib::Paths ps;
        ClipperLib::Paths contour_now; // Contours for calculation in ClipperLib::Path form, including contour and holes

        contour_now.push_back(contour);
        for (int i = 0; i < holes.size(); ++i)
        {
            contour_now.push_back(holes[i]);
        }

        // Cut with horizontal lines
        ClipperLib::cInt ymax = contour_now[0][0].Y, ymin = contour_now[0][0].Y;
        for (int i = 0; i < contour_now.size(); ++i)
        {
            for (int j = 0; j < contour_now[i].size(); ++j)
            {
                ymax = std::max(ymax, contour_now[i][j].Y);
                ymin = std::min(ymin, contour_now[i][j].Y);
            }
        }
        int num = ceil((ymax - ymin) / (scale * dis)) - 1;
        ClipperLib::cInt Dis = (ymax - ymin) / (num + 1);
        ClipperLib::cInt y0 = ymin + Dis / 2;

        std::vector<ClipperLib::IntPoint> intersection; // the set of intersections
        for (int i = 0; i < contour_now.size(); ++i)
        {
            for (int j = 0; j < contour_now[i].size(); ++j)
            {
                int I = ceil(1.0 * (contour_now[i][j].Y - y0) / Dis);
                int II = ceil(1.0 * (contour_now[i][(j + 1) % contour_now[i].size()].Y - y0) / Dis);
                for (int k = std::min(I, II); k < std::max(I, II); ++k)
                {
                    double lambda = 1.0 * (y0 + k * Dis - contour_now[i][j].Y) / (contour_now[i][(j + 1) % contour_now[i].size()].Y - contour_now[i][j].Y);
                    intersection.push_back(ClipperLib::IntPoint(ClipperLib::cInt(contour_now[i][j].X * (1.0 - lambda) + contour_now[i][(j + 1) % contour_now[i].size()].X * lambda), y0 + k * Dis));
                }
            }
        }

        // Generate Raster toolpaths
        std::sort(intersection.data(), intersection.data() + intersection.size(), cmp_Raster);
        for (int i = 0; i < intersection.size(); i += 2)
        {
            ClipperLib::Path p;
            p.push_back(intersection[i]);
            p.push_back(intersection[i + 1]);
            ps.push_back(p);
        }

        return ps;
    }

    // comparasion between points in Raster
    bool DirectionParalle::cmp_Raster(const ClipperLib::IntPoint &a, const ClipperLib::IntPoint &b)
    {
        if (a.Y == b.Y)
        {
            return a.X < b.X;
        }
        return a.Y < b.Y;
    }

    // Generate Zigzag toolpaths
    // dis is the distance between toolpaths; angle (rad) is the angle between toolpaths and the x-axis.
    ClipperLib::Paths DirectionParalle::Zigzag(const ClipperLib::Path &contour, const ClipperLib::Paths &holes, double dis, double scale)
    {
        ClipperLib::Paths ps;
        ClipperLib::Paths contour_now; // Contours for calculation in ClipperLib::Path form, including contour and holes

        contour_now.push_back(contour);
        for (int i = 0; i < holes.size(); ++i)
        {
            contour_now.push_back(holes[i]);
        }

        // Cut with horizontal lines
        ClipperLib::cInt ymax = contour_now[0][0].Y, ymin = contour_now[0][0].Y;
        for (int i = 0; i < contour_now.size(); ++i)
        {
            for (int j = 0; j < contour_now[i].size(); ++j)
            {
                ymax = std::max(ymax, contour_now[i][j].Y);
                ymin = std::min(ymin, contour_now[i][j].Y);
            }
        }
        int num = ceil((ymax - ymin) / (scale * dis)) - 1;
        ClipperLib::cInt Dis = (ymax - ymin) / (num + 1);
        ClipperLib::cInt y0 = ymin + Dis / 2;

        // Define the struct of InterPoint
        struct InterPoint
        {
            ClipperLib::IntPoint point;
            unsigned int idpath;
            unsigned int idpoint;
            InterPoint(ClipperLib::cInt x, ClipperLib::cInt y, unsigned int path, unsigned int point) : point(ClipperLib::IntPoint(x, y)), idpath(path), idpoint(point) {}
            bool operator<(const InterPoint &p) const
            {
                return DirectionParalle::cmp_Raster(this->point, p.point);
            }
            static bool nextto(const InterPoint &a, const InterPoint &b, int size)
            {
                // a and b is on the same continuous boundary and adjacent
                return (a.idpath == b.idpath) && ((a.idpoint == (b.idpoint + 1) % size) || (b.idpoint == (a.idpoint + 1) % size));
            }
        };
        std::vector<InterPoint> intersection; // the set of intersections

        int *size_pathinter = new int[contour_now.size()](); // number of intersections of every continuous boundary
        for (int i = 0; i < contour_now.size(); ++i)
        {
            unsigned int idpoint = 0;
            for (int j = 0; j < contour_now[i].size(); ++j)
            {
                int I = ceil(1.0 * (contour_now[i][j].Y - y0) / Dis);
                int II = ceil(1.0 * (contour_now[i][(j + 1) % contour_now[i].size()].Y - y0) / Dis);
                for (int k = std::min(I, II); k < std::max(I, II); ++k)
                {
                    double lambda = 1.0 * (y0 + k * Dis - contour_now[i][j].Y) / (contour_now[i][(j + 1) % contour_now[i].size()].Y - contour_now[i][j].Y);
                    intersection.push_back(InterPoint(ClipperLib::cInt(contour_now[i][j].X * (1.0 - lambda) + contour_now[i][(j + 1) % contour_now[i].size()].X * lambda), y0 + k * Dis, i, idpoint++));
                }
            }
            size_pathinter[i] = idpoint;
        }

        // Generate Zigzag toolpaths
        std::sort(intersection.data(), intersection.data() + intersection.size());
        std::vector<std::vector<InterPoint>> paths;
        for (int i = 0; i < intersection.size(); i += 2)
        {
            bool flag = false;
            for (int j = paths.size() - 1; j >= 0; --j)
            {
                if (InterPoint::nextto(intersection[i], paths[j][paths[j].size() - 1], size_pathinter[intersection[i].idpath]))
                {
                    paths[j].push_back(intersection[i]);
                    paths[j].push_back(intersection[i + 1]);
                    flag = true;
                    break;
                }
                if (InterPoint::nextto(intersection[i + 1], paths[j][paths[j].size() - 1], size_pathinter[intersection[i + 1].idpath]))
                {
                    paths[j].push_back(intersection[i + 1]);
                    paths[j].push_back(intersection[i]);
                    flag = true;
                    break;
                }
            }
            if (!flag)
            {
                std::vector<InterPoint> path_now;
                path_now.push_back(intersection[i]);
                path_now.push_back(intersection[i + 1]);
                paths.push_back(path_now);
            }
        }

        for (int i = 0; i < paths.size(); ++i)
        {
            ClipperLib::Path p;
            for (int j = 0; j < paths[i].size(); ++j)
            {
                p.push_back(paths[i][j].point);
            }
            ps.push_back(p);
        }

        return ps;
    }

    // Generate Zigzag toolpaths
    // dis is the distance between toolpaths; angle (rad) is the angle between toolpaths and the x-axis.
    paths DirectionParalle::Zigzag(const path &contour, const paths &holes, double dis, double angle /*=0*/)
    {
        if (!contour.length)
        {
            return paths();
        }

        // Rotate the boundaries
        path *contour_rotate = Curve::rotate(contour, -angle);
        paths *holes_rotate = Curve::rotate(holes, -angle);

        // Transform the rotated boundaries into ClipperLib::Path form
        double xmax = contour_rotate->x[0];
        double xmin = contour_rotate->x[0];
        double ymax = contour_rotate->y[0];
        double ymin = contour_rotate->y[0];
        for (int i = 1; i < contour_rotate->length; ++i)
        {
            xmax = (std::max)(xmax, contour_rotate->x[i]);
            xmin = (std::min)(xmin, contour_rotate->x[i]);
            ymax = (std::max)(ymax, contour_rotate->y[i]);
            ymin = (std::min)(ymin, contour_rotate->y[i]);
        }
        double delta_x = (xmax + xmin) * 0.5;
        double delta_y = (ymax + ymin) * 0.5;
        double scale = ContourParallel::ClipperBound / std::max(xmax - xmin, ymax - ymin);

        ClipperLib::Path Contour;
        ClipperLib::Paths Holes;
        for (int i = 0; i < contour_rotate->length; ++i)
        {
            Contour << ClipperLib::IntPoint(ContourParallel::double2cInt(contour_rotate->x[i], scale, delta_x), ContourParallel::double2cInt(contour_rotate->y[i], scale, delta_y));
        }
        Contour << Contour[0];

        for (int i_hole = 0; i_hole < holes_rotate->size(); ++i_hole)
        {
            Holes.push_back(ClipperLib::Path());
            for (int i = 0; i < (*holes_rotate)[i_hole].length; ++i)
            {
                Holes[Holes.size() - 1] << ClipperLib::IntPoint(ContourParallel::double2cInt((*holes_rotate)[i_hole].x[i], scale, delta_x), ContourParallel::double2cInt((*holes_rotate)[i_hole].y[i], scale, delta_y));
            }
            Holes[Holes.size() - 1] << Holes[Holes.size() - 1][0];
        }
        // Generate Zigzag toolpaths
        ClipperLib::Paths Solution = Zigzag(Contour, Holes, dis, scale);

        // Transform the Zigzag toolpaths into path form
        paths solution = ContourParallel::Paths2paths(Solution, scale, delta_x, delta_y);

        delete contour_rotate;
        delete holes_rotate;

        return *(Curve::rotate(solution, angle));
    }
}