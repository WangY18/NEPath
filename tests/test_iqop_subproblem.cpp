#include <NEPath/IqopSubproblem.h>
#include <NEPath/path.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace
{
constexpr double DERIVATIVE_STEP = 1.0e-7;      // Dimensionless central-difference diagnostic step.
constexpr double DERIVATIVE_TOLERANCE = 2.0e-6; // Allows second-order truncation plus double rounding.

nepath::path make_irregular_path()
{
    constexpr std::array<double, 6> x{0.0, 3.0, 3.5, 1.8, 1.0, -0.4};
    constexpr std::array<double, 6> y{0.0, 0.2, 2.4, 1.4, 3.1, 1.8};
    return nepath::path(x.data(), y.data(), static_cast<int>(x.size()));
}

nepath::NonEquidistantOptions qsl_options()
{
    nepath::NonEquidistantOptions options;
    options.delta = 0.4;
    options.alpha = 0.25;
    options.dot_delta = 0.8;
    options.ddot_delta = 0.2;
    options.optimize_Q = true;
    options.optimize_S = true;
    options.optimize_L = true;
    options.lambda_Q = 1.3;
    options.lambda_S = 0.7;
    options.lambda_L = 0.2;
    return options;
}

double derivative_for(const nepath::IqopSubproblem &subproblem, nepath::IqopConstraintBlock constraint, std::size_t row,
                      const nepath::IqopDecision &decision, std::size_t offset_column)
{
    nepath::IqopDecision minus = decision;
    nepath::IqopDecision plus = decision;
    minus.offsets.values.at(offset_column) -= DERIVATIVE_STEP;
    plus.offsets.values.at(offset_column) += DERIVATIVE_STEP;
    const double low = subproblem.evaluate(constraint, minus).at(row);
    const double high = subproblem.evaluate(constraint, plus).at(row);
    return (high - low) / (2.0 * DERIVATIVE_STEP);
}

std::size_t variable_size(nepath::IqopVariableBlock block, const nepath::IqopDecision &decision)
{
    switch (block)
    {
    case nepath::IqopVariableBlock::offsets:
        return decision.offsets.values.size();
    case nepath::IqopVariableBlock::edge_x:
        return decision.edge_x.values.size();
    case nepath::IqopVariableBlock::edge_y:
        return decision.edge_y.values.size();
    case nepath::IqopVariableBlock::edge_length:
        return decision.edge_length.values.size();
    case nepath::IqopVariableBlock::area_plus:
    case nepath::IqopVariableBlock::area_minus:
    case nepath::IqopVariableBlock::length:
    case nepath::IqopVariableBlock::quotient:
        return 1;
    }
    return 0;
}

void perturb(nepath::IqopDecision &decision, nepath::IqopVariableBlock block, std::size_t column, double amount)
{
    switch (block)
    {
    case nepath::IqopVariableBlock::offsets:
        decision.offsets.values.at(column) += amount;
        return;
    case nepath::IqopVariableBlock::edge_x:
        decision.edge_x.values.at(column) += amount;
        return;
    case nepath::IqopVariableBlock::edge_y:
        decision.edge_y.values.at(column) += amount;
        return;
    case nepath::IqopVariableBlock::edge_length:
        decision.edge_length.values.at(column) += amount;
        return;
    case nepath::IqopVariableBlock::area_plus:
        decision.area_plus.value += amount;
        return;
    case nepath::IqopVariableBlock::area_minus:
        decision.area_minus.value += amount;
        return;
    case nepath::IqopVariableBlock::length:
        decision.length.value += amount;
        return;
    case nepath::IqopVariableBlock::quotient:
        decision.quotient.value += amount;
        return;
    }
}
} // namespace

TEST_CASE("smoothness has four sparse three-point rows per vertex")
{
    const nepath::IqopGeometry geometry = nepath::IqopGeometry::build(make_irregular_path());
    const nepath::IqopSubproblem subproblem = nepath::IqopSubproblem::build(geometry, qsl_options());
    const nepath::IqopDecision decision = subproblem.initial_decision();

    const std::vector<double> values = subproblem.evaluate(nepath::IqopConstraintBlock::smoothness, decision);
    const std::vector<nepath::IqopJacobianEntry> jacobian =
        subproblem.jacobian(nepath::IqopConstraintBlock::smoothness, nepath::IqopVariableBlock::offsets, decision);

    REQUIRE(values.size() == 4 * geometry.vertex_count());
    REQUIRE(jacobian.size() == 12 * geometry.vertex_count());
    for (std::size_t row = 0; row < values.size(); ++row)
    {
        REQUIRE(std::count_if(jacobian.begin(), jacobian.end(), [row](const auto &entry) { return entry.row == row; }) == 3);
        REQUIRE(values.at(row) <= 0.0);
    }
}

TEST_CASE("area DC models bound true area and are tangent at reference")
{
    const nepath::IqopGeometry geometry = nepath::IqopGeometry::build(make_irregular_path());
    const nepath::IqopSubproblem subproblem = nepath::IqopSubproblem::build(geometry, qsl_options());
    const nepath::IqopDecision reference = subproblem.initial_decision();

    REQUIRE(subproblem.area_upper_model(reference) == Catch::Approx(subproblem.true_normalized_area(reference)));
    REQUIRE(subproblem.area_lower_model(reference) == Catch::Approx(subproblem.true_normalized_area(reference)));

    nepath::IqopDecision perturbed = reference;
    perturbed.offsets.values.at(0) -= 0.08;
    perturbed.offsets.values.at(1) += 0.06;
    perturbed.offsets.values.at(3) -= 0.04;

    REQUIRE(subproblem.area_lower_model(perturbed) <= subproblem.true_normalized_area(perturbed));
    REQUIRE(subproblem.area_upper_model(perturbed) >= subproblem.true_normalized_area(perturbed));
}

TEST_CASE("area model offset Jacobians match central differences")
{
    const nepath::IqopGeometry geometry = nepath::IqopGeometry::build(make_irregular_path());
    const nepath::IqopSubproblem subproblem = nepath::IqopSubproblem::build(geometry, qsl_options());
    nepath::IqopDecision decision = subproblem.initial_decision();
    decision.offsets.values.at(0) -= 0.05;
    decision.offsets.values.at(2) += 0.04;

    for (const nepath::IqopConstraintBlock block : {nepath::IqopConstraintBlock::area_plus, nepath::IqopConstraintBlock::area_minus})
    {
        const auto jacobian = subproblem.jacobian(block, nepath::IqopVariableBlock::offsets, decision);
        REQUIRE(jacobian.size() == geometry.vertex_count());
        for (const auto &entry : jacobian)
        {
            const double numerical = derivative_for(subproblem, block, entry.row, decision, entry.column);
            REQUIRE(entry.value == Catch::Approx(numerical).margin(DERIVATIVE_TOLERANCE));
        }
    }
}

TEST_CASE("strict initial epigraph is feasible for every active block")
{
    const nepath::IqopGeometry geometry = nepath::IqopGeometry::build(make_irregular_path());
    const nepath::IqopSubproblem subproblem = nepath::IqopSubproblem::build(geometry, qsl_options());
    const nepath::IqopDecision decision = subproblem.initial_decision();

    for (const nepath::IqopConstraintBlock block : subproblem.active_constraints())
    {
        for (const double value : subproblem.evaluate(block, decision))
        {
            REQUIRE(value < 0.0);
        }
    }
}

TEST_CASE("objective is the normalized weighted epigraph sum")
{
    const nepath::IqopGeometry geometry = nepath::IqopGeometry::build(make_irregular_path());
    const nepath::NonEquidistantOptions options = qsl_options();
    const nepath::IqopSubproblem subproblem = nepath::IqopSubproblem::build(geometry, options);
    const nepath::IqopDecision decision = subproblem.initial_decision();

    const double expected =
        options.lambda_Q * decision.quotient.value + options.lambda_S * decision.area_plus.value + options.lambda_L * decision.length.value;
    REQUIRE(subproblem.objective(decision) == Catch::Approx(expected));
}

TEST_CASE("every active sparse Jacobian block matches central differences")
{
    const nepath::IqopGeometry geometry = nepath::IqopGeometry::build(make_irregular_path());
    const nepath::IqopSubproblem subproblem = nepath::IqopSubproblem::build(geometry, qsl_options());
    nepath::IqopDecision decision = subproblem.initial_decision();
    decision.offsets.values.at(0) -= 0.03;
    decision.offsets.values.at(2) += 0.02;

    const std::array<nepath::IqopVariableBlock, 8> variable_blocks{
        nepath::IqopVariableBlock::offsets,     nepath::IqopVariableBlock::edge_x,    nepath::IqopVariableBlock::edge_y,
        nepath::IqopVariableBlock::edge_length, nepath::IqopVariableBlock::area_plus, nepath::IqopVariableBlock::area_minus,
        nepath::IqopVariableBlock::length,      nepath::IqopVariableBlock::quotient};

    for (const nepath::IqopConstraintBlock constraint : subproblem.active_constraints())
    {
        const std::size_t row_count = subproblem.evaluate(constraint, decision).size();
        for (const nepath::IqopVariableBlock variable : variable_blocks)
        {
            const std::size_t column_count = variable_size(variable, decision);
            const auto entries = subproblem.jacobian(constraint, variable, decision);
            std::vector<double> dense(row_count * column_count, 0.0);
            for (const auto &entry : entries)
            {
                dense.at(entry.row * column_count + entry.column) = entry.value;
            }
            for (std::size_t row = 0; row < row_count; ++row)
            {
                for (std::size_t column = 0; column < column_count; ++column)
                {
                    nepath::IqopDecision minus = decision;
                    nepath::IqopDecision plus = decision;
                    perturb(minus, variable, column, -DERIVATIVE_STEP);
                    perturb(plus, variable, column, DERIVATIVE_STEP);
                    const double numerical =
                        (subproblem.evaluate(constraint, plus).at(row) - subproblem.evaluate(constraint, minus).at(row)) /
                        (2.0 * DERIVATIVE_STEP);
                    REQUIRE(dense.at(row * column_count + column) == Catch::Approx(numerical).margin(DERIVATIVE_TOLERANCE));
                }
            }
        }
    }
}

TEST_CASE("subproblem rejects non-finite decisions")
{
    const nepath::IqopGeometry geometry = nepath::IqopGeometry::build(make_irregular_path());
    const nepath::IqopSubproblem subproblem = nepath::IqopSubproblem::build(geometry, qsl_options());
    nepath::IqopDecision decision = subproblem.initial_decision();
    decision.offsets.values.at(0) = std::numeric_limits<double>::quiet_NaN();

    REQUIRE_THROWS_AS(subproblem.objective(decision), nepath::InvalidIqopDecisionError);

    decision = subproblem.initial_decision();
    decision.quotient.value = std::numeric_limits<double>::infinity();
    REQUIRE_THROWS_AS(subproblem.evaluate(nepath::IqopConstraintBlock::iso_quotient, decision), nepath::InvalidIqopDecisionError);
}

TEST_CASE("subproblem rejects enabled objective terms with zero weight")
{
    const nepath::IqopGeometry geometry = nepath::IqopGeometry::build(make_irregular_path());
    nepath::NonEquidistantOptions options = qsl_options();
    options.lambda_Q = 0.0;

    REQUIRE_THROWS_AS(nepath::IqopSubproblem::build(geometry, options), nepath::InvalidIqopOptionsError);
}
