#pragma once

namespace nepath
{
constexpr const char *IQOP_SOLVER_CONFIGURATION_VERSION = "ipopt-lbfgs-v5";

// Desired KKT accuracy follows Ipopt's default and the Gurobi parity contract.
constexpr double IQOP_DESIRED_SOLVER_TOLERANCE = 1.0e-8;

// L-BFGS may plateau after primal feasibility; parity remains the correctness gate.
constexpr double IQOP_ACCEPTABLE_TOLERANCE_MULTIPLIER = 100'000.0;
constexpr double IQOP_ACCEPTABLE_CONSTRAINT_MULTIPLIER = 10.0;
constexpr int IQOP_ACCEPTABLE_ITERATION_COUNT = 5;
constexpr double IQOP_ACCEPTABLE_OBJECTIVE_CHANGE_TOLERANCE = 1.0e-8;

// Square root of desired KKT accuracy gives cone epigraphs resolvable interior slack.
constexpr double IQOP_INITIAL_INTERIOR_SLACK = 1.0e-4;

// Retains the explicit epigraph margin instead of Ipopt's O(n)-accumulating default.
constexpr double IQOP_INTERIOR_PUSH = 0.01 * IQOP_INITIAL_INTERIOR_SLACK;
constexpr double IQOP_INTERIOR_FRACTION = 0.01 * IQOP_INITIAL_INTERIOR_SLACK;

// Fixed cap: canonical 1,000-sample contours need >250 L-BFGS steps.
constexpr int IQOP_INNER_ITERATIONS_PER_SCP_STEP = 500;

// Bounds one convex subproblem while covering the canonical 1,000-vertex contour.
constexpr double IQOP_INNER_WALL_TIME_SECONDS = 240.0;

// Bounds the complete SCP solve independently of the caller's requested step count.
constexpr double IQOP_SCP_WALL_TIME_SECONDS = 600.0;
} // namespace nepath
