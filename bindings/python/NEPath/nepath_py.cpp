#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/ndarray.h>
// #include <nanobind/eigen/dense.h>
#include <NEPath/NEPath.h>
// Include NEPath headers

// The authors would like to sincerely thank Jelle Feringa for his significant contribution to python wrappers of NEPath.

using namespace nepath;
namespace nb = nanobind;
using namespace nb::literals;

// Helper function to convert numpy arrays to path
path numpy_to_path(nb::ndarray<double, nb::shape<-1>, nb::c_contig> x_array,
                   nb::ndarray<double, nb::shape<-1>, nb::c_contig> y_array)
{
    size_t length = x_array.shape(0);
    if (length != y_array.shape(0))
    {
        throw std::runtime_error("x and y arrays must have the same length");
    }

    path p;
    p.length = static_cast<int>(length);
    p.x = new double[p.length];
    p.y = new double[p.length];

    const double *x_data = x_array.data();
    const double *y_data = y_array.data();

    for (int i = 0; i < p.length; ++i)
    {
        p.x[i] = x_data[i];
        p.y[i] = y_data[i];
    }

    return p;
}

// Helper function to convert path to Python tuple of numpy arrays
std::tuple<nb::ndarray<nb::numpy, double, nb::shape<-1>>,
           nb::ndarray<nb::numpy, double, nb::shape<-1>>>
path_to_numpy(const path &p)
{
    size_t shape[1] = {static_cast<size_t>(p.length)};

    // Create numpy arrays
    auto x_array = nb::ndarray<nb::numpy, double, nb::shape<-1>>(
        p.x, 1, shape, nb::handle());
    auto y_array = nb::ndarray<nb::numpy, double, nb::shape<-1>>(
        p.y, 1, shape, nb::handle());

    return std::make_tuple(x_array, y_array);
}

NB_MODULE(_nepath, m)
{
    m.doc() = "NEPath Python bindings for toolpath planning";

    // Bind ConnectAlgorithm enum
    nb::enum_<ConnectAlgorithm>(m, "ConnectAlgorithm")
        .value("none", ConnectAlgorithm::none)
        .value("cfs", ConnectAlgorithm::cfs, "Connected Fermat Spiral")
        .value("dfs", ConnectAlgorithm::dfs, "Connected DFS")
        .export_values();

    // Bind OptimizationAlgorithm enum
    nb::enum_<OptimizationAlgorithm>(m, "OptimizationAlgorithm")
        .value("none", OptimizationAlgorithm::none, "No solver")
        .value("ipopt", OptimizationAlgorithm::ipopt, "IPOPT solver")
        .value("gurobi", OptimizationAlgorithm::gurobi, "Gurobi solver")
        .export_values();

    // Bind path struct
    nb::class_<path>(m, "Path")
        .def(nb::init<>())
        .def(nb::init<const path &>())
        .def_rw("length", &path::length, "Number of waypoints")
        .def("xmax", &path::xmax, "Find the max x-coordinate")
        .def("xmin", &path::xmin, "Find the min x-coordinate")
        .def("ymax", &path::ymax, "Find the max y-coordinate")
        .def("ymin", &path::ymin, "Find the min y-coordinate")
        .def("get_arrays", [](const path &p)
             {
            // Return copies of the arrays as Python lists
            nb::list x_list;
            nb::list y_list;
            for (int i = 0; i < p.length; ++i) {
                x_list.append(p.x[i]);
                y_list.append(p.y[i]);
            }
            return nb::make_tuple(x_list, y_list); }, "Get x and y coordinates as Python lists")
        .def("set_arrays", [](path &p, nb::ndarray<double, nb::shape<-1>, nb::c_contig> x_array, nb::ndarray<double, nb::shape<-1>, nb::c_contig> y_array)
             {
            size_t length = x_array.shape(0);
            if (length != y_array.shape(0)) {
                throw std::runtime_error("x and y arrays must have the same length");
            }

            // Clean up old data
            if (p.x) delete[] p.x;
            if (p.y) delete[] p.y;

            // Allocate and copy new data
            p.length = static_cast<int>(length);
            p.x = new double[p.length];
            p.y = new double[p.length];

            const double* x_data = x_array.data();
            const double* y_data = y_array.data();

            for (int i = 0; i < p.length; ++i) {
                p.x[i] = x_data[i];
                p.y[i] = y_data[i];
            } }, "x"_a, "y"_a, "Set x and y coordinates from numpy arrays")
        .def_static("from_arrays", [](nb::ndarray<double, nb::shape<-1>, nb::c_contig> x_array, nb::ndarray<double, nb::shape<-1>, nb::c_contig> y_array)
                    { return numpy_to_path(x_array, y_array); }, "x"_a, "y"_a, "Create a Path from numpy arrays");

    // Bind DirectParallelOptions
    nb::class_<DirectParallelOptions>(m, "DirectParallelOptions")
        .def(nb::init<>())
        .def_rw("delta", &DirectParallelOptions::delta, "Line width")
        .def_rw("angle", &DirectParallelOptions::angle, "Angle (rad) between toolpaths and x-axis");

    // Bind ContourParallelOptions
    nb::class_<ContourParallelOptions>(m, "ContourParallelOptions")
        .def(nb::init<>())
        .def_rw("delta", &ContourParallelOptions::delta, "Line width")
        .def_rw("wash", &ContourParallelOptions::wash, "Enable resampling")
        .def_rw("washdis", &ContourParallelOptions::washdis, "Resampling distance")
        .def_rw("num_least", &ContourParallelOptions::num_least, "Minimum number of waypoints")
        .def_rw("connector", &ContourParallelOptions::connector, "Connection algorithm");

    // Bind NonEquidistantOptions
    nb::class_<NonEquidistantOptions>(m, "NonEquidistantOptions")
        .def(nb::init<>())
        .def_rw("delta", &NonEquidistantOptions::delta, "Upper bound of delta_i")
        .def_rw("alpha", &NonEquidistantOptions::alpha, "Lower bound of delta_i")
        .def_rw("dot_delta", &NonEquidistantOptions::dot_delta, "Upper bound of dot(delta_i)")
        .def_rw("ddot_delta", &NonEquidistantOptions::ddot_delta, "Upper bound of ddot(delta_i)")
        .def_rw("optimize_Q", &NonEquidistantOptions::optimize_Q, "Optimize isoperimetric quotient")
        .def_rw("optimize_S", &NonEquidistantOptions::optimize_S, "Optimize area")
        .def_rw("optimize_L", &NonEquidistantOptions::optimize_L, "Optimize length")
        .def_rw("lambda_Q", &NonEquidistantOptions::lambda_Q, "Weighting coefficient for Q")
        .def_rw("lambda_S", &NonEquidistantOptions::lambda_S, "Weighting coefficient for S")
        .def_rw("lambda_L", &NonEquidistantOptions::lambda_L, "Weighting coefficient for L")
        .def_rw("epsilon", &NonEquidistantOptions::epsilon, "Maximum error of offsetting distances")
        .def_rw("step_max", &NonEquidistantOptions::step_max, "Maximum iteration steps")
        .def_rw("wash", &NonEquidistantOptions::wash, "Enable resampling")
        .def_rw("washdis", &NonEquidistantOptions::washdis, "Resampling distance")
        .def_rw("num_least", &NonEquidistantOptions::num_least, "Minimum number of waypoints")
        .def_rw("connector", &NonEquidistantOptions::connector, "Connection algorithm")
        .def_rw("optimizer", &NonEquidistantOptions::optimizer, "Optimization solver");

    // Bind UnderFillSolution
    nb::class_<UnderFillSolution>(m, "UnderFillSolution")
        .def(nb::init<>())
        .def_rw("nx", &UnderFillSolution::nx, "Length of xs")
        .def_rw("ny", &UnderFillSolution::ny, "Length of ys")
        .def_rw("underfillrate", &UnderFillSolution::underfillrate, "Underfill rate")
        .def("get_xs", [](const UnderFillSolution &sol)
             { return std::vector<double>(sol.xs, sol.xs + sol.nx); }, "Get xs array")
        .def("get_ys", [](const UnderFillSolution &sol)
             { return std::vector<double>(sol.ys, sol.ys + sol.ny); }, "Get ys array")
        .def("get_map_slice", [](const UnderFillSolution &sol)
             {
            std::vector<std::vector<bool>> result(sol.nx, std::vector<bool>(sol.ny));
            for (int i = 0; i < sol.nx; ++i) {
                for (int j = 0; j < sol.ny; ++j) {
                    result[i][j] = sol.map_slice[i][j];
                }
            }
            return result; }, "Get map_slice as 2D list")
        .def("get_map_delta", [](const UnderFillSolution &sol)
             {
            std::vector<std::vector<bool>> result(sol.nx, std::vector<bool>(sol.ny));
            for (int i = 0; i < sol.nx; ++i) {
                for (int j = 0; j < sol.ny; ++j) {
                    result[i][j] = sol.map_delta[i][j];
                }
            }
            return result; }, "Get map_delta as 2D list")
        .def("clear", &UnderFillSolution::clear, "Clear the solution");

    // Bind SharpTurnSolution
    nb::class_<SharpTurnSolution>(m, "SharpTurnSolution")
        .def(nb::init<>())
        .def_rw("length", &SharpTurnSolution::length, "Number of waypoints")
        .def_rw("radius", &SharpTurnSolution::radius, "Radius of the circle")
        .def_rw("threshold", &SharpTurnSolution::threshold, "Threshold of area to determine sharp corners")
        .def_rw("close", &SharpTurnSolution::close, "Whether the toolpath is closed")
        .def("get_area_percent", [](const SharpTurnSolution &sol)
             { return std::vector<double>(sol.AreaPercent, sol.AreaPercent + sol.length); }, "Get AreaPercent array")
        .def("get_sharp_turn", [](const SharpTurnSolution &sol)
             { return std::vector<bool>(sol.SharpTurn, sol.SharpTurn + sol.length); }, "Get SharpTurn array")
        .def("clear", &SharpTurnSolution::clear, "Clear the solution");

    // Bind NEPathPlanner class
    nb::class_<NEPathPlanner>(m, "NEPathPlanner")
        .def(nb::init<>())
        .def("set_contour",
             nb::overload_cast<const path &, bool, double, int>(&NEPathPlanner::set_contour),
             "contour"_a, "wash"_a = true, "washdis"_a = 0.2, "num_least"_a = 50,
             "Set the contour (outer boundary) of the slice")
        .def("set_contour", [](NEPathPlanner &self, nb::ndarray<double, nb::shape<-1>, nb::c_contig> x_array, nb::ndarray<double, nb::shape<-1>, nb::c_contig> y_array, bool wash = true, double washdis = 0.2, int num_least = 50)
             {
                 path p = numpy_to_path(x_array, y_array);
                 self.set_contour(p, wash, washdis, num_least); }, "x"_a, "y"_a, "wash"_a = true, "washdis"_a = 0.2, "num_least"_a = 50, "Set the contour from numpy arrays")
        .def("addhole", nb::overload_cast<const path &, bool, double, int>(&NEPathPlanner::addhole), "hole"_a, "wash"_a = true, "washdis"_a = 0.2, "num_least"_a = 50, "Add a new hole (inner boundary) onto the slice")
        .def("addhole", [](NEPathPlanner &self, nb::ndarray<double, nb::shape<-1>, nb::c_contig> x_array, nb::ndarray<double, nb::shape<-1>, nb::c_contig> y_array, bool wash = true, double washdis = 0.2, int num_least = 50)
             {
                 path p = numpy_to_path(x_array, y_array);
                 self.addhole(p, wash, washdis, num_least); }, "x"_a, "y"_a, "wash"_a = true, "washdis"_a = 0.2, "num_least"_a = 50, "Add a hole from numpy arrays")
        .def("addholes", &NEPathPlanner::addholes, "holes"_a, "wash"_a = true, "washdis"_a = 0.2, "num_least"_a = 50, "Add multiple holes (inner boundaries) onto the slice")
        .def("tool_compensate", &NEPathPlanner::tool_compensate, "opts"_a, "Offset the contour and holes of the slice with a distance")
        .def("Raster", &NEPathPlanner::Raster, "opts"_a, "Generate Raster toolpath")
        .def("Zigzag", &NEPathPlanner::Zigzag, "opts"_a, "Generate Zigzag toolpath")
        .def("CP", &NEPathPlanner::CP, "opts"_a, "Generate CP (Contour Parallel) toolpath")
        .def("IQOP", &NEPathPlanner::IQOP, "opts"_a, "log"_a = true, "Generate IQOP (Isoperimetric Quotient Optimization) toolpath")
        .def_rw("contour", &NEPathPlanner::contour, "The contour of the slice")
        .def_rw("holes", &NEPathPlanner::holes, "The holes of the slice");

    // Bind Curve static methods
    nb::class_<Curve>(m, "Curve")
        .def_static("UnderFill", &Curve::UnderFill,
                    "contour"_a, "holes"_a, "ps"_a, "delta"_a, "reratio"_a,
                    "Calculate the underfill")
        .def_static("UnderFillRate", &Curve::UnderFillRate,
                    "contour"_a, "holes"_a, "ps"_a, "delta"_a, "reratio"_a,
                    "Calculate the underfill rate")
        .def_static("SharpTurn_Invariant", &Curve::SharpTurn_Invariant,
                    "p"_a, "radius"_a, "threshold"_a = 0.3, "close"_a = false, "washdis"_a = -1.0,
                    "Calculate the sharp corners")
        .def_static("SharpTurnNum_Invariant", &Curve::SharpTurnNum_Invariant,
                    "p"_a, "radius"_a, "threshold"_a = 0.3, "close"_a = false, "washdis"_a = -1.0,
                    "Calculate the number of sharp corners")
        .def_static("wash_dis",
                    nb::overload_cast<const path &, double, int>(&Curve::wash_dis),
                    "p"_a, "dis"_a, "num_least"_a = 50,
                    "Resample the path with a distance approximately equal to dis, where the minimum number of waypoints is num_least")
        .def_static("AreaCal", &Curve::AreaCal,
                    "x"_a, "y"_a, "length"_a,
                    "Calculate area enclosed by path")
        .def_static("TotalLength", &Curve::TotalLength,
                    "x"_a, "y"_a, "length"_a, "poly"_a = true,
                    "Calculate length of path");

    // // Bind FileAgent static methods
    // nb::class_<FileAgent>(m, "FileAgent")
    //     .def_static("read_csv", &FileAgent::read_csv,
    //                 "filename"_a,
    //                 "Read a path from a CSV file")
    //     .def_static("write_csv",
    //                 nb::overload_cast<const path &, char const *>(&FileAgent::write_csv),
    //                 "path"_a, "filename"_a,
    //                 "Write a path to a CSV file")
    //     .def_static("write_csv",
    //                 nb::overload_cast<const paths &, char const *, char const *>(&FileAgent::write_csv),
    //                 "paths"_a, "filename_pre"_a, "filename_post"_a = nullptr,
    //                 "Write paths to CSV files with prefix and suffix")
    //     .def_static("delete_AllFiles", &FileAgent::delete_AllFiles,
    //                 "path"_a,
    //                 "Delete all files in the folder")
    //     .def_static("mkdir", &FileAgent::mkdir,
    //                 "path"_a, "clear"_a = false,
    //                 "Create a directory");
}
