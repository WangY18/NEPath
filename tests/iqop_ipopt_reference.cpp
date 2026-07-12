#include <NEPath/Basic.h>
#include <NEPath/IqopIfopt.h>
#include <NEPath/IqopScp.h>
#include <NEPath/path.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr double SOLVER_TOLERANCE = 1.0e-8;
constexpr int SOLVER_ITERATION_LIMIT = 500;

struct Metrics final
{
    double normalized_area;
    double normalized_perimeter;
    double quotient;
};

template <std::size_t Count> nepath::path make_path(const std::array<double, Count> &x, const std::array<double, Count> &y)
{
    return nepath::path(x.data(), y.data(), static_cast<int>(x.size()));
}

nepath::path make_fixture(const std::string &fixture)
{
    if (fixture == "irregular")
    {
        constexpr std::array<double, 6> x{0.0, 3.0, 3.5, 1.8, 1.0, -0.4};
        constexpr std::array<double, 6> y{0.0, 0.2, 2.4, 1.4, 3.1, 1.8};
        return make_path(x, y);
    }
    if (fixture == "convex")
    {
        constexpr std::array<double, 8> x{0.0, 4.0, 4.5, 4.0, 2.5, 0.5, -0.5, 0.0};
        constexpr std::array<double, 8> y{0.0, 0.0, 1.0, 3.0, 4.0, 3.5, 2.0, 0.7};
        return make_path(x, y);
    }
    if (fixture == "slender")
    {
        constexpr std::array<double, 7> x{0.0, 5.0, 6.0, 5.2, 3.0, 0.5, -0.5};
        constexpr std::array<double, 7> y{0.0, 0.2, 1.0, 1.8, 2.2, 1.8, 1.0};
        return make_path(x, y);
    }
    throw std::invalid_argument("unknown geometry fixture");
}

nepath::NonEquidistantOptions make_options(const std::string &topology)
{
    if (topology != "q" && topology != "s" && topology != "l" && topology != "qs" && topology != "ql" && topology != "sl" &&
        topology != "qsl")
    {
        throw std::invalid_argument("unknown objective topology");
    }
    nepath::NonEquidistantOptions options;
    options.delta = 0.35;
    options.alpha = 0.2;
    options.dot_delta = 1.0;
    options.ddot_delta = 0.4;
    options.optimize_Q = topology.find('q') != std::string::npos;
    options.optimize_S = topology.find('s') != std::string::npos;
    options.optimize_L = topology.find('l') != std::string::npos;
    options.lambda_Q = 1.0;
    options.lambda_S = 0.4;
    options.lambda_L = 0.2;
    options.epsilon = 0.1;
    options.step_max = 8;
    return options;
}

Metrics metrics(const nepath::IqopGeometry &geometry, const nepath::NormalizedIqopOffsets &offsets, double offset_scale)
{
    double twice_area = 0.0;
    double perimeter = 0.0;
    for (std::size_t i = 0; i < geometry.vertex_count(); ++i)
    {
        const std::size_t next = (i + 1) % geometry.vertex_count();
        const double current_offset = offset_scale * offsets.values.at(i);
        const double next_offset = offset_scale * offsets.values.at(next);
        const double current_x = geometry.point(i).x.value() + geometry.normal_x(i).value() * current_offset;
        const double current_y = geometry.point(i).y.value() + geometry.normal_y(i).value() * current_offset;
        const double next_x = geometry.point(next).x.value() + geometry.normal_x(next).value() * next_offset;
        const double next_y = geometry.point(next).y.value() + geometry.normal_y(next).value() * next_offset;
        twice_area += current_x * next_y - current_y * next_x;
        perimeter += std::hypot(next_x - current_x, next_y - current_y);
    }
    const double area = 0.5 * twice_area;
    return Metrics{area / geometry.area().value(), perimeter / geometry.perimeter().value(),
                   perimeter * perimeter / (4.0 * nepath::pi * area)};
}

void write_result(double objective, double maximum_constraint_violation, const Metrics &geometry_metrics, std::size_t scp_iterations,
                  const nepath::NormalizedIqopOffsets &offsets)
{
    std::cout << "objective,max_constraint_violation,normalized_area,normalized_perimeter,geometric_q,scp_iterations";
    for (std::size_t i = 0; i < offsets.values.size(); ++i)
    {
        std::cout << ",offset_" << i;
    }
    std::cout << '\n';
    std::cout << std::setprecision(17) << objective << ',' << maximum_constraint_violation << ',' << geometry_metrics.normalized_area << ','
              << geometry_metrics.normalized_perimeter << ',' << geometry_metrics.quotient << ',' << scp_iterations;
    for (const double offset : offsets.values)
    {
        std::cout << ',' << offset;
    }
    std::cout << '\n';
}
} // namespace

int main(int argc, char **argv)
{
    try
    {
        if (argc != 4)
        {
            throw std::invalid_argument("usage: iqop_ipopt_reference <q|s|l|qs|ql|sl|qsl> <fixed|scp> <irregular|convex|slender>");
        }
        const std::string topology = argv[1];
        const std::string mode = argv[2];
        const std::string fixture = argv[3];
        const nepath::NonEquidistantOptions options = make_options(topology);
        const nepath::path input = make_fixture(fixture);
        const nepath::IqopGeometry geometry = nepath::IqopGeometry::build(input);

        if (mode == "fixed")
        {
            const nepath::IqopSubproblem subproblem = nepath::IqopSubproblem::build(geometry, options);
            const nepath::IqopSolveResult result = nepath::solve_iqop_subproblem(
                subproblem, nepath::IqopSolverSettings::build(SOLVER_TOLERANCE, SOLVER_ITERATION_LIMIT, false));
            write_result(result.objective, result.maximum_constraint_violation, metrics(geometry, result.decision.offsets, options.delta),
                         1, result.decision.offsets);
            return 0;
        }
        if (mode == "scp")
        {
            const nepath::IqopScpResult result = nepath::solve_iqop_with_ipopt(input, options, false);
            if (result.iterations.empty())
            {
                throw std::runtime_error("Ipopt SCP returned no iterations");
            }
            const nepath::IqopScpIteration &last = result.iterations.back();
            write_result(last.objective, last.maximum_constraint_violation,
                         Metrics{last.normalized_area, last.normalized_perimeter, last.geometric_quotient}, result.iterations.size(),
                         result.normalized_offsets);
            return 0;
        }
        throw std::invalid_argument("mode must be fixed or scp");
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
