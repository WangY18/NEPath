#pragma once

#include <NEPath/IqopBuildIdentity.h>
#include <NEPath/IqopSubproblem.h>

#include <chrono>
#include <string>

namespace nepath
{
using IqopSolverWallTime = std::chrono::duration<double>;

struct IqopSolverSettings final
{
    [[nodiscard]] static IqopSolverSettings build(double tolerance, int maximum_iterations, bool verbose);
    [[nodiscard]] static IqopSolverSettings build_bounded(double tolerance, int maximum_iterations, IqopSolverWallTime wall_time_limit,
                                                          bool verbose);

    double tolerance;
    int maximum_iterations;
    IqopSolverWallTime wall_time_limit;
    bool verbose;

  private:
    IqopSolverSettings(double tolerance_value, int maximum_iterations_value, IqopSolverWallTime wall_time_limit_value,
                       bool verbose_value) noexcept;
};

enum class IqopTerminationStatus
{
    desired,
    acceptable
};

struct IqopSolveResult final
{
    IqopDecision decision;
    double objective;
    double maximum_constraint_violation;
    int iterations;
    double wall_time_seconds;
    IqopTerminationStatus termination_status;
    IqopBuildIdentity build_identity;
};

[[nodiscard]] IqopSolveResult solve_iqop_subproblem(const IqopSubproblem &subproblem, const IqopSolverSettings &settings);
} // namespace nepath
