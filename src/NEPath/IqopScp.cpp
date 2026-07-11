#include <NEPath/Basic.h>
#include <NEPath/IqopScp.h>
#include <NEPath/IqopSolverDefaults.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <utility>

namespace
{
struct GeometricMetrics final
{
    double normalized_area;
    double normalized_perimeter;
    double quotient;
};

[[nodiscard]] GeometricMetrics geometric_metrics(const nepath::IqopGeometry &geometry, const nepath::NormalizedIqopOffsets &offsets,
                                                 double offset_scale)
{
    double twice_area = 0.0;
    double perimeter = 0.0;
    const std::size_t count = geometry.vertex_count();
    for (std::size_t i = 0; i < count; ++i)
    {
        const std::size_t next = (i + 1) % count;
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
    const double quotient = perimeter * perimeter / (4.0 * nepath::pi * area);
    return GeometricMetrics{area / geometry.area().value(), perimeter / geometry.perimeter().value(), quotient};
}
} // namespace

namespace nepath
{
IqopScpResult solve_iqop_with_ipopt(const path &input, const NonEquidistantOptions &options, bool verbose)
{
    return solve_iqop_with_ipopt_bounded(input, options, verbose, IqopSolverWallTime(IQOP_SCP_WALL_TIME_SECONDS));
}

IqopScpResult solve_iqop_with_ipopt_bounded(const path &input, const NonEquidistantOptions &options, bool verbose,
                                            IqopSolverWallTime total_wall_time_limit)
{
    if (!std::isfinite(total_wall_time_limit.count()) || total_wall_time_limit <= IqopSolverWallTime::zero())
    {
        throw InvalidIqopSolverSettingsError("IQOP SCP wall-time limit must be finite and positive");
    }
    const auto solve_started = std::chrono::steady_clock::now();
    const IqopGeometry geometry = IqopGeometry::build(input);
    NormalizedIqopOffsets reference{std::vector<double>(geometry.vertex_count(), 0.5 * (1.0 + options.alpha))};
    const IqopSubproblem first_subproblem = IqopSubproblem::build(geometry, options, reference);
    std::vector<IqopScpIteration> history;
    history.reserve(static_cast<std::size_t>(std::max(options.step_max, 0)));
    bool converged = false;
    IqopBuildIdentity build_identity = build_iqop_identity(first_subproblem);

    for (int iteration = 0; iteration < options.step_max; ++iteration)
    {
        const IqopSubproblem subproblem = iteration == 0 ? first_subproblem : IqopSubproblem::build(geometry, options, reference);
        const IqopSolverWallTime elapsed = std::chrono::steady_clock::now() - solve_started;
        const IqopSolverWallTime remaining = total_wall_time_limit - elapsed;
        if (remaining <= IqopSolverWallTime::zero())
        {
            throw IqopSolveTimeLimitError("IQOP SCP exceeded its total wall-time limit, input " + build_identity.input_sha256);
        }
        const IqopSolverWallTime subproblem_limit = std::min(IqopSolverWallTime(IQOP_INNER_WALL_TIME_SECONDS), remaining);
        const IqopSolverSettings settings =
            IqopSolverSettings::build_bounded(IQOP_DESIRED_SOLVER_TOLERANCE, IQOP_INNER_ITERATIONS_PER_SCP_STEP, subproblem_limit, verbose);
        const IqopSolveResult solved = solve_iqop_subproblem(subproblem, settings);
        double normalized_change = 0.0;
        for (std::size_t i = 0; i < geometry.vertex_count(); ++i)
        {
            normalized_change = std::max(normalized_change, std::abs(solved.decision.offsets.values.at(i) - reference.values.at(i)));
        }
        const double physical_change = options.delta * normalized_change;
        const GeometricMetrics metrics = geometric_metrics(geometry, solved.decision.offsets, options.delta);
        history.push_back(IqopScpIteration{iteration + 1, solved.objective, solved.maximum_constraint_violation, physical_change,
                                           metrics.normalized_area, metrics.normalized_perimeter, metrics.quotient, solved.iterations,
                                           solved.wall_time_seconds, solved.termination_status});
        reference = solved.decision.offsets;
        build_identity = solved.build_identity;

        if ((!options.optimize_Q && !options.optimize_S) || (iteration > 1 && physical_change < options.epsilon))
        {
            converged = true;
            break;
        }
    }

    std::vector<OffsetLength> physical_offsets;
    physical_offsets.reserve(reference.values.size());
    for (const double offset : reference.values)
    {
        physical_offsets.emplace_back(options.delta * offset);
    }
    return IqopScpResult{std::move(reference), std::move(physical_offsets), std::move(history), converged, std::move(build_identity)};
}
} // namespace nepath
