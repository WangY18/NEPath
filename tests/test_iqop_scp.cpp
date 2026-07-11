#include <NEPath/IqopScp.h>
#include <NEPath/IqopSolverDefaults.h>
#include <NEPath/NEPathPlanner.h>
#include <NEPath/path.h>

#include <catch2/catch_test_macros.hpp>

#include <array>

namespace
{
nepath::path make_irregular_path()
{
    constexpr std::array<double, 6> x{0.0, 3.0, 3.5, 1.8, 1.0, -0.4};
    constexpr std::array<double, 6> y{0.0, 0.2, 2.4, 1.4, 3.1, 1.8};
    return nepath::path(x.data(), y.data(), static_cast<int>(x.size()));
}

nepath::NonEquidistantOptions options_for(bool quotient, bool area, bool length)
{
    nepath::NonEquidistantOptions options;
    options.delta = 0.35;
    options.alpha = 0.2;
    options.dot_delta = 1.0;
    options.ddot_delta = 0.4;
    options.optimize_Q = quotient;
    options.optimize_S = area;
    options.optimize_L = length;
    options.lambda_Q = 1.0;
    options.lambda_S = 0.4;
    options.lambda_L = 0.2;
    options.epsilon = 0.1;
    options.step_max = 8;
    return options;
}
} // namespace

TEST_CASE("length-only SCP matches the Gurobi single-subproblem stopping rule")
{
    const nepath::NonEquidistantOptions options = options_for(false, false, true);

    const nepath::IqopScpResult result = nepath::solve_iqop_with_ipopt(make_irregular_path(), options, false);

    REQUIRE(result.iterations.size() == 1);
    REQUIRE(result.normalized_offsets.values.size() == 6);
    REQUIRE(result.physical_offsets.size() == 6);
}

TEST_CASE("quotient SCP rebuilds area models to a fixed point")
{
    const nepath::NonEquidistantOptions options = options_for(true, false, false);

    const nepath::IqopScpResult result = nepath::solve_iqop_with_ipopt(make_irregular_path(), options, false);

    REQUIRE(result.converged);
    REQUIRE(result.iterations.size() >= 3);
    REQUIRE(result.iterations.size() <= static_cast<std::size_t>(options.step_max));
    REQUIRE(result.iterations.back().physical_offset_change <= options.epsilon);
    REQUIRE(result.iterations.back().maximum_constraint_violation <= 1.0e-7);
    for (const nepath::IqopScpIteration &iteration : result.iterations)
    {
        REQUIRE(iteration.ipopt_iterations <= nepath::IQOP_INNER_ITERATIONS_PER_SCP_STEP);
        REQUIRE((iteration.termination_status == nepath::IqopTerminationStatus::desired ||
                 iteration.termination_status == nepath::IqopTerminationStatus::acceptable));
    }
}

TEST_CASE("public planner delegates Ipopt IQOP to the SCP formulation")
{
    constexpr std::array<double, 4> x{0.0, 2.0, 2.0, 0.0};
    constexpr std::array<double, 4> y{0.0, 0.0, 2.0, 2.0};
    nepath::NEPathPlanner planner;
    planner.set_contour(x.data(), y.data(), static_cast<int>(x.size()), false);
    nepath::NonEquidistantOptions options = options_for(false, false, true);
    options.delta = 0.25;
    options.alpha = 0.2;
    options.optimizer = nepath::OptimizationAlgorithm::ipopt;
    options.wash = true;
    options.washdis = 2.0;
    options.num_least = 4;

    const nepath::paths result = planner.IQOP(options, false);

    REQUIRE_FALSE(result.empty());
    REQUIRE(result.front().length > 0);
}

TEST_CASE("SCP rejects a non-positive outer iteration limit before solving")
{
    nepath::NonEquidistantOptions options = options_for(true, false, false);
    options.step_max = 0;

    REQUIRE_THROWS_AS(nepath::solve_iqop_with_ipopt(make_irregular_path(), options, false), nepath::InvalidIqopOptionsError);
}

TEST_CASE("bounded SCP rejects an invalid total wall-time budget")
{
    const nepath::NonEquidistantOptions options = options_for(true, false, false);

    REQUIRE_THROWS_AS(nepath::solve_iqop_with_ipopt_bounded(make_irregular_path(), options, false, nepath::IqopSolverWallTime::zero()),
                      nepath::InvalidIqopSolverSettingsError);
}
