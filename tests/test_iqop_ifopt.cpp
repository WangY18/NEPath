#include <NEPath/IqopIfopt.h>
#include <NEPath/path.h>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
constexpr double SOLVER_TOLERANCE = 1.0e-8;
constexpr double FEASIBILITY_GATE = 10.0 * SOLVER_TOLERANCE;
constexpr std::size_t BOUND_PUSH_REGRESSION_VERTEX_COUNT = 256;
constexpr int BOUND_PUSH_REGRESSION_ITERATION_GATE = 40;

nepath::path make_square()
{
    constexpr std::array<double, 4> x{0.0, 2.0, 2.0, 0.0};
    constexpr std::array<double, 4> y{0.0, 0.0, 2.0, 2.0};
    return nepath::path(x.data(), y.data(), static_cast<int>(x.size()));
}

nepath::NonEquidistantOptions length_options()
{
    nepath::NonEquidistantOptions options;
    options.delta = 0.25;
    options.alpha = 0.2;
    options.dot_delta = 1.0;
    options.ddot_delta = 0.5;
    options.optimize_Q = false;
    options.optimize_S = false;
    options.optimize_L = true;
    options.lambda_L = 1.0;
    return options;
}

nepath::NonEquidistantOptions objective_options(bool quotient, bool area, bool length)
{
    nepath::NonEquidistantOptions options = length_options();
    options.optimize_Q = quotient;
    options.optimize_S = area;
    options.optimize_L = length;
    options.lambda_Q = 1.0;
    options.lambda_S = 0.6;
    options.lambda_L = 0.2;
    return options;
}

nepath::path make_lobed_path()
{
    constexpr double radius = 15.0;
    constexpr double lobe_amplitude = 0.1;
    constexpr double lobe_count = 10.0;
    const double two_pi = 2.0 * std::acos(-1.0);
    std::vector<double> x(BOUND_PUSH_REGRESSION_VERTEX_COUNT);
    std::vector<double> y(BOUND_PUSH_REGRESSION_VERTEX_COUNT);
    for (std::size_t i = 0; i < BOUND_PUSH_REGRESSION_VERTEX_COUNT; ++i)
    {
        const double theta = two_pi * static_cast<double>(i) / static_cast<double>(BOUND_PUSH_REGRESSION_VERTEX_COUNT);
        const double local_radius = radius * (1.0 + lobe_amplitude * std::cos(lobe_count * theta));
        x[i] = local_radius * std::cos(theta);
        y[i] = local_radius * std::sin(theta);
    }
    return nepath::path(x.data(), y.data(), static_cast<int>(x.size()));
}

void require_solve_improves(bool quotient, bool area, bool length)
{
    const nepath::IqopGeometry geometry = nepath::IqopGeometry::build(make_square());
    const nepath::IqopSubproblem subproblem = nepath::IqopSubproblem::build(geometry, objective_options(quotient, area, length));
    const nepath::IqopSolverSettings settings = nepath::IqopSolverSettings::build(SOLVER_TOLERANCE, 500, false);

    const nepath::IqopSolveResult result = nepath::solve_iqop_subproblem(subproblem, settings);

    REQUIRE(result.objective < subproblem.objective(subproblem.initial_decision()));
    REQUIRE(result.maximum_constraint_violation <= FEASIBILITY_GATE);
    REQUIRE((result.termination_status == nepath::IqopTerminationStatus::desired ||
             result.termination_status == nepath::IqopTerminationStatus::acceptable));
    REQUIRE(result.build_identity.ifopt_revision != "unavailable");
}
} // namespace

TEST_CASE("ifopt solves every active IQOP objective topology")
{
    SECTION("quotient")
    {
        require_solve_improves(true, false, false);
    }
    SECTION("area")
    {
        require_solve_improves(false, true, false);
    }
    SECTION("quotient and area")
    {
        require_solve_improves(true, true, false);
    }
    SECTION("quotient and length")
    {
        require_solve_improves(true, false, true);
    }
    SECTION("area and length")
    {
        require_solve_improves(false, true, true);
    }
    SECTION("quotient area and length")
    {
        require_solve_improves(true, true, true);
    }
}

TEST_CASE("ifopt solves the length epigraph subproblem")
{
    const nepath::IqopGeometry geometry = nepath::IqopGeometry::build(make_square());
    const nepath::IqopSubproblem subproblem = nepath::IqopSubproblem::build(geometry, length_options());
    const double initial_objective = subproblem.objective(subproblem.initial_decision());
    const nepath::IqopSolverSettings settings = nepath::IqopSolverSettings::build(SOLVER_TOLERANCE, 500, false);

    const nepath::IqopSolveResult result = nepath::solve_iqop_subproblem(subproblem, settings);

    REQUIRE(result.objective < initial_objective);
    REQUIRE(result.maximum_constraint_violation <= FEASIBILITY_GATE);
    REQUIRE(result.decision.length.value > 0.0);
    for (const double offset : result.decision.offsets.values)
    {
        REQUIRE(offset >= length_options().alpha - FEASIBILITY_GATE);
        REQUIRE(offset <= 1.0 + FEASIBILITY_GATE);
    }
}

TEST_CASE("ifopt preserves a feasible edge-epigraph start on a sampled contour")
{
    nepath::NonEquidistantOptions options = objective_options(true, true, false);
    options.delta = 1.0;
    options.alpha = 0.5;
    options.ddot_delta = 0.1;
    options.lambda_S = 1.0;
    const nepath::IqopGeometry geometry = nepath::IqopGeometry::build(make_lobed_path());
    const nepath::IqopSubproblem subproblem = nepath::IqopSubproblem::build(geometry, options);
    const nepath::IqopSolverSettings settings = nepath::IqopSolverSettings::build(SOLVER_TOLERANCE, 300, false);

    const nepath::IqopSolveResult result = nepath::solve_iqop_subproblem(subproblem, settings);

    REQUIRE(result.maximum_constraint_violation <= FEASIBILITY_GATE);
    REQUIRE(result.iterations <= BOUND_PUSH_REGRESSION_ITERATION_GATE);
}

TEST_CASE("ifopt solver settings reject an invalid wall-time budget")
{
    REQUIRE_THROWS_AS(nepath::IqopSolverSettings::build_bounded(SOLVER_TOLERANCE, 100, nepath::IqopSolverWallTime::zero(), false),
                      nepath::InvalidIqopSolverSettingsError);
    REQUIRE_THROWS_AS(nepath::IqopSolverSettings::build_bounded(SOLVER_TOLERANCE, 100,
                                                                nepath::IqopSolverWallTime(std::numeric_limits<double>::infinity()), false),
                      nepath::InvalidIqopSolverSettingsError);
}
