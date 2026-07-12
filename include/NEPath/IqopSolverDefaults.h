#pragma once

namespace nepath
{
constexpr const char *IQOP_SOLVER_CONFIGURATION_VERSION = "ipopt-lbfgs-v8";

// Desired KKT accuracy follows Ipopt's default and the Gurobi parity contract.
constexpr double IQOP_DESIRED_SOLVER_TOLERANCE = 1.0e-8;

// L-BFGS may plateau after primal feasibility; parity remains the correctness gate.
constexpr double IQOP_ACCEPTABLE_TOLERANCE_MULTIPLIER = 10'000.0;
constexpr double IQOP_ACCEPTABLE_CONSTRAINT_MULTIPLIER = 10.0;
constexpr int IQOP_ACCEPTABLE_ITERATION_COUNT = 5;

// Square root of desired KKT accuracy gives cone epigraphs resolvable interior slack.
constexpr double IQOP_INITIAL_INTERIOR_SLACK = 1.0e-4;

// Retains the explicit epigraph margin instead of Ipopt's O(n)-accumulating default.
constexpr double IQOP_INTERIOR_PUSH = 0.01 * IQOP_INITIAL_INTERIOR_SLACK;
constexpr double IQOP_INTERIOR_FRACTION = 0.01 * IQOP_INITIAL_INTERIOR_SLACK;

// Fixed cap; unlike the former policy, it does not grow with the SCP step count.
constexpr int IQOP_INNER_ITERATIONS_PER_SCP_STEP = 1'000;

// Bounds one convex subproblem while covering the canonical 1,000-vertex contour.
constexpr double IQOP_INNER_WALL_TIME_SECONDS = 1'800.0;

// Bounds the complete SCP solve independently of the caller's requested step count.
constexpr double IQOP_SCP_WALL_TIME_SECONDS = 1'800.0;
} // namespace nepath
