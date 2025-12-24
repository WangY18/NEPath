#include <NEPath/NEPathPlanner.h>

namespace nepath
{
    // Set the contour (outer boundary) of the slice.
    // If wash==true, the output CP toolpaths would be resampled with a uniformly-distributed distance no more than wash_dis, and the number of waypoints are no less than num_least.
    void NEPathPlanner::set_contour(const double *x, const double *y, int length, bool wash /*=true*/, double washdis /*=0.2*/, int num_least /*=50*/)
    {
        contour.clear_with_delete();
        if (wash)
        {
            path p = Curve::wash_dis(x, y, length, washdis);
            contour.steal(p);
        }
        else
        {
            contour.copy_with_new(x, y, length);
        }
    }

    // Set the contour (outer boundary) of the slice.
    // If wash==true, the output CP toolpaths would be resampled with a uniformly-distributed distance no more than wash_dis, and the number of waypoints are no less than num_least.
    void NEPathPlanner::set_contour(const path &contour_new, bool wash /*=true*/, double washdis /*=0.2*/, int num_least /*=50*/)
    {
        contour.clear_with_delete();
        if (wash)
        {
            path p = Curve::wash_dis(contour_new, washdis);
            contour.steal(p);
        }
        else
        {
            contour.copy_with_new(contour_new);
        }
    }

    // Add a new hole (inner boundary) onto the slice.
    // If wash==true, the output CP toolpaths would be resampled with a uniformly-distributed distance no more than wash_dis, and the number of waypoints are no less than num_least.
    void NEPathPlanner::addhole(const double *x, const double *y, int length, bool wash /*=true*/, double washdis /*=0.2*/, int num_least /*=50*/)
    {
        holes.push_back(path());
        if (wash)
        {
            path p = Curve::wash_dis(x, y, length, washdis);
            holes[holes.size() - 1].steal(p);
        }
        else
        {
            holes[holes.size() - 1].copy_with_new(x, y, length);
        }
    }

    // Add a new hole (inner boundary) onto the slice.
    // If wash==true, the output CP toolpaths would be resampled with a uniformly-distributed distance no more than wash_dis, and the number of waypoints are no less than num_least.
    void NEPathPlanner::addhole(const path &hole_new, bool wash /*=true*/, double washdis /*=0.2*/, int num_least /*=50*/)
    {
        holes.push_back(path());
        if (wash)
        {
            path p = Curve::wash_dis(hole_new, washdis);
            holes[holes.size() - 1].steal(p);
        }
        else
        {
            holes[holes.size() - 1].copy_with_new(hole_new);
        }
    }

    // Add some new holes (inner boundaries) onto the slice.
    // If wash==true, the output CP toolpaths would be resampled with a uniformly-distributed distance no more than wash_dis, and the number of waypoints are no less than num_least.
    void NEPathPlanner::addholes(const paths &holes_new, bool wash /*=true*/, double washdis /*=0.2*/, int num_least /*=50*/)
    {
        for (int i = 0; i < holes_new.size(); ++i)
        {
            holes.push_back(path());
            if (wash)
            {
                path p = Curve::wash_dis(holes_new[i], washdis);
                holes[holes.size() - 1].steal(p);
            }
            else
            {
                holes[holes.size() - 1].copy_with_new(holes_new[i]);
            }
        }
    }

#if defined(IncludeIpopt) && (IncludeIpopt != 0)
    // Generate IQOP toolpath
    paths NEPathPlanner::IQOP(const NonEquidistantOptions &opts, bool log /*=true*/)
    {
        NonEquidistant NE(log);
        if (opts.connector == ConnectAlgorithm::none)
        {
            return NE.NEpaths(contour, holes, opts);
        }
        paths ps;
        if (opts.connector == ConnectAlgorithm::cfs)
        {
            ps.push_back(NE.NEpaths_CFS(contour, holes, opts));
        }
        else if (opts.connector == ConnectAlgorithm::dfs)
        {
            ps.push_back(NE.NEpaths_DFS(contour, holes, opts));
        }
        return ps;
    }
#endif

    // Generate Raster toolpath
    paths NEPathPlanner::Raster(const DirectParallelOptions &opts)
    {
        return DirectionParalle::Raster(contour, holes, opts.delta, opts.angle);
    }

    // Generate Zigzag toolpath
    paths NEPathPlanner::Zigzag(const DirectParallelOptions &opts)
    {
        return DirectionParalle::Zigzag(contour, holes, opts.delta, opts.angle);
    }

    // Generate CP toolpath
    paths NEPathPlanner::CP(const ContourParallelOptions &opts)
    {
        if (opts.connector == ConnectAlgorithm::none)
        {
            return ContourParallel::Contour_Parallel(contour, holes, opts.delta, opts.wash, opts.washdis, opts.num_least);
        }
        paths ps;
        if (opts.connector == ConnectAlgorithm::cfs)
        {
            ps.push_back(ContourParallel::Contour_Parallel_CFS(contour, holes, opts.delta, opts.wash, opts.washdis, opts.num_least));
        }
        else if (opts.connector == ConnectAlgorithm::dfs)
        {
            ps.push_back(ContourParallel::Contour_Parallel_DFS(contour, holes, opts.delta, opts.wash, opts.washdis, opts.num_least));
        }
        return ps;
    }

    // Offset the contour and holes of the slice with a distance
    // opts.delta>0 means offsetting the path inside; dis<0 means offsetting the path outside.
    // If opts.wash==true, the output paths would be resampled with a uniformly-distributed distance no more than opts.wash_dis, and the number of waypoints are no less than opts.num_least.
    paths NEPathPlanner::tool_compensate(const ContourParallelOptions &opts)
    {
        return ContourParallel::tool_compensate(contour, holes, opts.delta, opts.wash, opts.washdis, opts.num_least);
    }
}