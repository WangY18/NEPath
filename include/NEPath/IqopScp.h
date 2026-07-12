#pragma once

#include <NEPath/IqopIfopt.h>
#include <NEPath/IqopTypes.h>
#include <NEPath/PlanningOptions.h>
#include <NEPath/path.h>

#include <vector>

namespace nepath
{
struct IqopScpIteration final
{
    int index;
    double objective;
    double maximum_constraint_violation;
    double physical_offset_change;
    double normalized_area;
    double normalized_perimeter;
    double geometric_quotient;
    int ipopt_iterations;
    double wall_time_seconds;
    IqopTerminationStatus termination_status;
};

struct IqopScpResult final
{
    NormalizedIqopOffsets normalized_offsets;
    std::vector<OffsetLength> physical_offsets;
    std::vector<IqopScpIteration> iterations;
    bool converged;
    IqopBuildIdentity build_identity;
};

[[nodiscard]] IqopScpResult solve_iqop_with_ipopt(const path &input, const NonEquidistantOptions &options, bool verbose);
[[nodiscard]] IqopScpResult solve_iqop_with_ipopt_bounded(const path &input, const NonEquidistantOptions &options, bool verbose,
                                                          IqopSolverWallTime total_wall_time_limit);
} // namespace nepath
