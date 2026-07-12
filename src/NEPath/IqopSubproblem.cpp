#include <NEPath/Basic.h>
#include <NEPath/IqopSolverDefaults.h>
#include <NEPath/IqopSubproblem.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <sstream>
#include <utility>

namespace
{
const double FOUR_PI = 4.0 * nepath::pi;

void require_finite(double value, const char *name)
{
    if (!std::isfinite(value))
    {
        throw nepath::InvalidIqopOptionsError(std::string("IQOP option is non-finite: ") + name);
    }
}

void validate_options(const nepath::NonEquidistantOptions &options)
{
    require_finite(options.delta, "delta");
    require_finite(options.alpha, "alpha");
    require_finite(options.dot_delta, "dot_delta");
    require_finite(options.ddot_delta, "ddot_delta");
    require_finite(options.lambda_Q, "lambda_Q");
    require_finite(options.lambda_S, "lambda_S");
    require_finite(options.lambda_L, "lambda_L");
    require_finite(options.epsilon, "epsilon");
    if (options.delta <= 0.0)
    {
        throw nepath::InvalidIqopOptionsError("IQOP delta must be positive");
    }
    if (options.alpha < 0.0 || options.alpha > 1.0)
    {
        throw nepath::InvalidIqopOptionsError("IQOP alpha must lie in [0, 1]");
    }
    if (options.dot_delta < 0.0 || options.ddot_delta < 0.0)
    {
        throw nepath::InvalidIqopOptionsError("IQOP smoothness bounds must be non-negative");
    }
    if (options.lambda_Q < 0.0 || options.lambda_S < 0.0 || options.lambda_L < 0.0)
    {
        throw nepath::InvalidIqopOptionsError("IQOP objective weights must be non-negative");
    }
    if ((options.optimize_Q && options.lambda_Q == 0.0) || (options.optimize_S && options.lambda_S == 0.0) ||
        (options.optimize_L && options.lambda_L == 0.0))
    {
        throw nepath::InvalidIqopOptionsError("every enabled IQOP objective term requires a positive weight");
    }
    if (!options.optimize_Q && !options.optimize_S && !options.optimize_L)
    {
        throw nepath::InvalidIqopOptionsError("IQOP requires at least one active objective term");
    }
    if (options.epsilon <= 0.0 || options.step_max <= 0)
    {
        throw nepath::InvalidIqopOptionsError("IQOP SCP tolerance and iteration limit must be positive");
    }
}

double upper_area_term(double coefficient, double current, double next, double reference, double reference_next)
{
    const double sum = current + next;
    const double reference_sum = reference + reference_next;
    if (coefficient >= 0.0)
    {
        return 0.5 * coefficient *
               (sum * sum - 2.0 * reference * current - 2.0 * reference_next * next + reference * reference +
                reference_next * reference_next);
    }
    return 0.5 * coefficient * (2.0 * reference_sum * sum - current * current - next * next - reference_sum * reference_sum);
}

double lower_area_term(double coefficient, double current, double next, double reference, double reference_next)
{
    const double sum = current + next;
    const double reference_sum = reference + reference_next;
    if (coefficient >= 0.0)
    {
        return 0.5 * coefficient * (2.0 * reference_sum * sum - current * current - next * next - reference_sum * reference_sum);
    }
    return 0.5 * coefficient *
           (sum * sum - 2.0 * reference * current - 2.0 * reference_next * next + reference * reference + reference_next * reference_next);
}
} // namespace

namespace nepath
{
IqopSubproblem::IqopSubproblem(IqopGeometry geometry, NonEquidistantOptions options, NormalizedIqopOffsets reference_offsets,
                               std::vector<double> area_linear_coefficients, std::vector<double> area_normal_coefficients,
                               IqopDecision initial_decision, std::vector<IqopConstraintBlock> active_constraints) noexcept
    : geometry_(std::move(geometry)), options_(options), reference_offsets_(std::move(reference_offsets)),
      area_linear_coefficients_(std::move(area_linear_coefficients)), area_normal_coefficients_(std::move(area_normal_coefficients)),
      initial_decision_(std::move(initial_decision)), active_constraints_(std::move(active_constraints))
{
}

IqopSubproblem IqopSubproblem::build(const IqopGeometry &geometry, const NonEquidistantOptions &options)
{
    const double initial_offset = 0.5 * (1.0 + options.alpha);
    return build(geometry, options, NormalizedIqopOffsets{std::vector<double>(geometry.vertex_count(), initial_offset)});
}

IqopSubproblem IqopSubproblem::build(const IqopGeometry &geometry, const NonEquidistantOptions &options,
                                     NormalizedIqopOffsets reference_offsets)
{
    validate_options(options);
    const std::size_t count = geometry.vertex_count();
    if (reference_offsets.values.size() != count)
    {
        throw InvalidIqopDecisionError("IQOP reference offset count does not match path vertices");
    }
    for (const double offset : reference_offsets.values)
    {
        if (!std::isfinite(offset) || offset < options.alpha || offset > 1.0)
        {
            throw InvalidIqopDecisionError("IQOP normalized reference offset lies outside [alpha, 1]");
        }
    }

    const double area_scale = geometry.area().value();
    const double offset_scale = options.delta;
    std::vector<double> area_linear_coefficients(count);
    std::vector<double> area_normal_coefficients(count);
    for (std::size_t i = 0; i < count; ++i)
    {
        const std::size_t previous = (i + count - 1) % count;
        const std::size_t next = (i + 1) % count;
        area_linear_coefficients[i] =
            0.5 *
            ((geometry.point(next).y.value() - geometry.point(previous).y.value()) * geometry.normal_x(i).value() -
             (geometry.point(next).x.value() - geometry.point(previous).x.value()) * geometry.normal_y(i).value()) *
            offset_scale / area_scale;
        area_normal_coefficients[i] = 0.5 *
                                      (geometry.normal_y(next).value() * geometry.normal_x(i).value() -
                                       geometry.normal_x(next).value() * geometry.normal_y(i).value()) *
                                      offset_scale * offset_scale / area_scale;
    }

    std::vector<IqopConstraintBlock> active_constraints{IqopConstraintBlock::smoothness};
    if (options.optimize_Q || options.optimize_L)
    {
        active_constraints.push_back(IqopConstraintBlock::edge_decomposition);
        active_constraints.push_back(IqopConstraintBlock::edge_norm);
        active_constraints.push_back(IqopConstraintBlock::length_sum);
    }
    if (options.optimize_S)
    {
        active_constraints.push_back(IqopConstraintBlock::area_plus);
    }
    if (options.optimize_Q)
    {
        active_constraints.push_back(IqopConstraintBlock::area_minus);
        active_constraints.push_back(IqopConstraintBlock::iso_quotient);
    }

    IqopDecision initial;
    initial.offsets = reference_offsets;
    if (options.optimize_Q || options.optimize_L)
    {
        initial.edge_x.values.resize(count);
        initial.edge_y.values.resize(count);
        initial.edge_length.values.resize(count);
        const double perimeter_scale = geometry.perimeter().value();
        double length_sum = 0.0;
        for (std::size_t i = 0; i < count; ++i)
        {
            const std::size_t next = (i + 1) % count;
            const double current_offset = offset_scale * reference_offsets.values[i];
            const double next_offset = offset_scale * reference_offsets.values[next];
            const double edge_x = (geometry.point(i).x.value() + geometry.normal_x(i).value() * current_offset -
                                   geometry.point(next).x.value() - geometry.normal_x(next).value() * next_offset) /
                                  perimeter_scale;
            const double edge_y = (geometry.point(i).y.value() + geometry.normal_y(i).value() * current_offset -
                                   geometry.point(next).y.value() - geometry.normal_y(next).value() * next_offset) /
                                  perimeter_scale;
            initial.edge_x.values[i] = std::abs(edge_x) + IQOP_INITIAL_INTERIOR_SLACK;
            initial.edge_y.values[i] = std::abs(edge_y) + IQOP_INITIAL_INTERIOR_SLACK;
            initial.edge_length.values[i] = std::hypot(initial.edge_x.values[i], initial.edge_y.values[i]) + IQOP_INITIAL_INTERIOR_SLACK;
            length_sum += initial.edge_length.values[i];
        }
        initial.length.value = length_sum + IQOP_INITIAL_INTERIOR_SLACK;
    }

    IqopSubproblem provisional(geometry, options, reference_offsets, area_linear_coefficients, area_normal_coefficients, initial,
                               active_constraints);
    if (options.optimize_S)
    {
        initial.area_plus.value = provisional.area_upper_model(initial) + IQOP_INITIAL_INTERIOR_SLACK;
    }
    if (options.optimize_Q)
    {
        initial.area_minus.value = provisional.area_lower_model(initial) - IQOP_INITIAL_INTERIOR_SLACK;
        if (initial.area_minus.value <= IQOP_INITIAL_INTERIOR_SLACK)
        {
            throw InvalidIqopDecisionError("IQOP initial lower area model is not strictly positive");
        }
        const double quotient_floor = initial.length.value * initial.length.value * geometry.perimeter().value() *
                                      geometry.perimeter().value() / (FOUR_PI * geometry.area().value() * initial.area_minus.value);
        initial.quotient.value = std::max(1.0, quotient_floor) + IQOP_INITIAL_INTERIOR_SLACK;
    }

    return IqopSubproblem(geometry, options, std::move(reference_offsets), std::move(area_linear_coefficients),
                          std::move(area_normal_coefficients), std::move(initial), std::move(active_constraints));
}

void IqopSubproblem::validate_decision(const IqopDecision &decision) const
{
    const std::size_t count = geometry_.vertex_count();
    if (decision.offsets.values.size() != count)
    {
        throw InvalidIqopDecisionError("IQOP decision offset count does not match path vertices");
    }
    if (has_edges() &&
        (decision.edge_x.values.size() != count || decision.edge_y.values.size() != count || decision.edge_length.values.size() != count))
    {
        throw InvalidIqopDecisionError("IQOP decision edge block count does not match path vertices");
    }
    const auto require_finite_block = [](const std::vector<double> &values, const char *name)
    {
        for (std::size_t i = 0; i < values.size(); ++i)
        {
            if (!std::isfinite(values[i]))
            {
                std::ostringstream message;
                message << "IQOP decision contains a non-finite " << name << " at index " << i;
                throw InvalidIqopDecisionError(message.str());
            }
        }
    };
    require_finite_block(decision.offsets.values, "offset");
    if (has_edges())
    {
        require_finite_block(decision.edge_x.values, "edge-x epigraph");
        require_finite_block(decision.edge_y.values, "edge-y epigraph");
        require_finite_block(decision.edge_length.values, "edge-length epigraph");
    }
    const auto require_finite_scalar = [](double value, const char *name)
    {
        if (!std::isfinite(value))
        {
            throw InvalidIqopDecisionError(std::string("IQOP decision contains a non-finite ") + name);
        }
    };
    if (has_area_plus())
    {
        require_finite_scalar(decision.area_plus.value, "upper-area epigraph");
    }
    if (has_area_minus())
    {
        require_finite_scalar(decision.area_minus.value, "lower-area epigraph");
    }
    if (has_length())
    {
        require_finite_scalar(decision.length.value, "length epigraph");
    }
    if (has_quotient())
    {
        require_finite_scalar(decision.quotient.value, "quotient epigraph");
    }
}

double IqopSubproblem::true_normalized_area(const IqopDecision &decision) const
{
    validate_decision(decision);
    double area = 1.0;
    for (std::size_t i = 0; i < geometry_.vertex_count(); ++i)
    {
        const std::size_t next = (i + 1) % geometry_.vertex_count();
        area += area_linear_coefficients_[i] * decision.offsets.values[i];
        area += area_normal_coefficients_[i] * decision.offsets.values[i] * decision.offsets.values[next];
    }
    return area;
}

double IqopSubproblem::area_upper_model(const IqopDecision &decision) const
{
    validate_decision(decision);
    double area = 1.0;
    for (std::size_t i = 0; i < geometry_.vertex_count(); ++i)
    {
        const std::size_t next = (i + 1) % geometry_.vertex_count();
        area += area_linear_coefficients_[i] * decision.offsets.values[i];
        area += upper_area_term(area_normal_coefficients_[i], decision.offsets.values[i], decision.offsets.values[next],
                                reference_offsets_.values[i], reference_offsets_.values[next]);
    }
    return area;
}

double IqopSubproblem::area_lower_model(const IqopDecision &decision) const
{
    validate_decision(decision);
    double area = 1.0;
    for (std::size_t i = 0; i < geometry_.vertex_count(); ++i)
    {
        const std::size_t next = (i + 1) % geometry_.vertex_count();
        area += area_linear_coefficients_[i] * decision.offsets.values[i];
        area += lower_area_term(area_normal_coefficients_[i], decision.offsets.values[i], decision.offsets.values[next],
                                reference_offsets_.values[i], reference_offsets_.values[next]);
    }
    return area;
}

std::vector<double> IqopSubproblem::area_model_gradient(const IqopDecision &decision, bool upper) const
{
    validate_decision(decision);
    const std::size_t count = geometry_.vertex_count();
    std::vector<double> gradient = area_linear_coefficients_;
    for (std::size_t i = 0; i < count; ++i)
    {
        const std::size_t next = (i + 1) % count;
        const double coefficient = area_normal_coefficients_[i];
        const double current = decision.offsets.values[i];
        const double following = decision.offsets.values[next];
        const double reference = reference_offsets_.values[i];
        const double reference_next = reference_offsets_.values[next];
        const bool convex_sum_form = upper ? coefficient >= 0.0 : coefficient < 0.0;
        if (convex_sum_form)
        {
            gradient[i] += coefficient * (current + following - reference);
            gradient[next] += coefficient * (current + following - reference_next);
        }
        else
        {
            const double reference_sum = reference + reference_next;
            gradient[i] += coefficient * (reference_sum - current);
            gradient[next] += coefficient * (reference_sum - following);
        }
    }
    return gradient;
}

std::vector<double> IqopSubproblem::evaluate(IqopConstraintBlock block, const IqopDecision &decision) const
{
    validate_decision(decision);
    const std::size_t count = geometry_.vertex_count();
    const double perimeter = geometry_.perimeter().value();
    const double offset_scale = options_.delta;
    std::vector<double> values;
    switch (block)
    {
    case IqopConstraintBlock::smoothness:
        values.reserve(4 * count);
        for (std::size_t i = 0; i < count; ++i)
        {
            const std::size_t previous = (i + count - 1) % count;
            const std::size_t next = (i + 1) % count;
            const double edge = geometry_.edge_length(i).value();
            const double previous_edge = geometry_.edge_length(previous).value();
            const double current_offset = offset_scale * decision.offsets.values[i];
            const double previous_offset = offset_scale * decision.offsets.values[previous];
            const double next_offset = offset_scale * decision.offsets.values[next];
            const double first = -edge * edge * previous_offset + (edge * edge - previous_edge * previous_edge) * current_offset +
                                 previous_edge * previous_edge * next_offset;
            const double first_bound = options_.dot_delta * previous_edge * edge * (previous_edge + edge);
            values.push_back((first - first_bound) / (perimeter * perimeter * perimeter));
            values.push_back((-first - first_bound) / (perimeter * perimeter * perimeter));
            const double second = edge * previous_offset - (previous_edge + edge) * current_offset + previous_edge * next_offset;
            const double second_bound = 0.5 * options_.ddot_delta * previous_edge * edge * (previous_edge + edge);
            values.push_back((second - second_bound) / (perimeter * perimeter));
            values.push_back((-second - second_bound) / (perimeter * perimeter));
        }
        return values;
    case IqopConstraintBlock::edge_decomposition:
        values.reserve(4 * count);
        for (std::size_t i = 0; i < count; ++i)
        {
            const std::size_t next = (i + 1) % count;
            const double current_offset = offset_scale * decision.offsets.values[i];
            const double next_offset = offset_scale * decision.offsets.values[next];
            const double edge_x = (geometry_.point(i).x.value() + geometry_.normal_x(i).value() * current_offset -
                                   geometry_.point(next).x.value() - geometry_.normal_x(next).value() * next_offset) /
                                  perimeter;
            const double edge_y = (geometry_.point(i).y.value() + geometry_.normal_y(i).value() * current_offset -
                                   geometry_.point(next).y.value() - geometry_.normal_y(next).value() * next_offset) /
                                  perimeter;
            values.push_back(edge_x - decision.edge_x.values[i]);
            values.push_back(-edge_x - decision.edge_x.values[i]);
            values.push_back(edge_y - decision.edge_y.values[i]);
            values.push_back(-edge_y - decision.edge_y.values[i]);
        }
        return values;
    case IqopConstraintBlock::edge_norm:
        values.reserve(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            values.push_back(decision.edge_x.values[i] * decision.edge_x.values[i] + decision.edge_y.values[i] * decision.edge_y.values[i] -
                             decision.edge_length.values[i] * decision.edge_length.values[i]);
        }
        return values;
    case IqopConstraintBlock::length_sum:
        return {std::accumulate(decision.edge_length.values.begin(), decision.edge_length.values.end(), 0.0) - decision.length.value};
    case IqopConstraintBlock::area_plus:
        return {area_upper_model(decision) - decision.area_plus.value};
    case IqopConstraintBlock::area_minus:
        return {decision.area_minus.value - area_lower_model(decision)};
    case IqopConstraintBlock::iso_quotient:
    {
        const double area_to_perimeter_squared = geometry_.area().value() / (perimeter * perimeter);
        return {decision.length.value * decision.length.value -
                FOUR_PI * area_to_perimeter_squared * decision.area_minus.value * decision.quotient.value};
    }
    }
    throw InvalidIqopDecisionError("Unknown IQOP constraint block");
}

std::vector<IqopJacobianEntry> IqopSubproblem::jacobian(IqopConstraintBlock constraint, IqopVariableBlock variable,
                                                        const IqopDecision &decision) const
{
    validate_decision(decision);
    const std::size_t count = geometry_.vertex_count();
    const double perimeter = geometry_.perimeter().value();
    const double offset_scale = options_.delta;
    std::vector<IqopJacobianEntry> entries;
    if (constraint == IqopConstraintBlock::smoothness && variable == IqopVariableBlock::offsets)
    {
        entries.reserve(12 * count);
        for (std::size_t i = 0; i < count; ++i)
        {
            const std::size_t previous = (i + count - 1) % count;
            const std::size_t next = (i + 1) % count;
            const double edge = geometry_.edge_length(i).value();
            const double previous_edge = geometry_.edge_length(previous).value();
            const double first_scale = offset_scale / (perimeter * perimeter * perimeter);
            const double second_scale = offset_scale / (perimeter * perimeter);
            const std::array<double, 3> first{-edge * edge * first_scale, (edge * edge - previous_edge * previous_edge) * first_scale,
                                              previous_edge * previous_edge * first_scale};
            const std::array<double, 3> second{edge * second_scale, -(previous_edge + edge) * second_scale, previous_edge * second_scale};
            const std::array<std::size_t, 3> columns{previous, i, next};
            for (std::size_t local = 0; local < columns.size(); ++local)
            {
                entries.push_back({4 * i, columns[local], first[local]});
                entries.push_back({4 * i + 1, columns[local], -first[local]});
                entries.push_back({4 * i + 2, columns[local], second[local]});
                entries.push_back({4 * i + 3, columns[local], -second[local]});
            }
        }
        return entries;
    }
    if (constraint == IqopConstraintBlock::edge_decomposition)
    {
        if (variable == IqopVariableBlock::offsets)
        {
            entries.reserve(8 * count);
            for (std::size_t i = 0; i < count; ++i)
            {
                const std::size_t next = (i + 1) % count;
                const double current_x = geometry_.normal_x(i).value() * offset_scale / perimeter;
                const double next_x = -geometry_.normal_x(next).value() * offset_scale / perimeter;
                const double current_y = geometry_.normal_y(i).value() * offset_scale / perimeter;
                const double next_y = -geometry_.normal_y(next).value() * offset_scale / perimeter;
                entries.push_back({4 * i, i, current_x});
                entries.push_back({4 * i, next, next_x});
                entries.push_back({4 * i + 1, i, -current_x});
                entries.push_back({4 * i + 1, next, -next_x});
                entries.push_back({4 * i + 2, i, current_y});
                entries.push_back({4 * i + 2, next, next_y});
                entries.push_back({4 * i + 3, i, -current_y});
                entries.push_back({4 * i + 3, next, -next_y});
            }
        }
        else if (variable == IqopVariableBlock::edge_x)
        {
            for (std::size_t i = 0; i < count; ++i)
            {
                entries.push_back({4 * i, i, -1.0});
                entries.push_back({4 * i + 1, i, -1.0});
            }
        }
        else if (variable == IqopVariableBlock::edge_y)
        {
            for (std::size_t i = 0; i < count; ++i)
            {
                entries.push_back({4 * i + 2, i, -1.0});
                entries.push_back({4 * i + 3, i, -1.0});
            }
        }
        return entries;
    }
    if (constraint == IqopConstraintBlock::edge_norm)
    {
        entries.reserve(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            if (variable == IqopVariableBlock::edge_x)
            {
                entries.push_back({i, i, 2.0 * decision.edge_x.values[i]});
            }
            else if (variable == IqopVariableBlock::edge_y)
            {
                entries.push_back({i, i, 2.0 * decision.edge_y.values[i]});
            }
            else if (variable == IqopVariableBlock::edge_length)
            {
                entries.push_back({i, i, -2.0 * decision.edge_length.values[i]});
            }
        }
        return entries;
    }
    if (constraint == IqopConstraintBlock::length_sum)
    {
        if (variable == IqopVariableBlock::edge_length)
        {
            for (std::size_t i = 0; i < count; ++i)
            {
                entries.push_back({0, i, 1.0});
            }
        }
        else if (variable == IqopVariableBlock::length)
        {
            entries.push_back({0, 0, -1.0});
        }
        return entries;
    }
    if ((constraint == IqopConstraintBlock::area_plus || constraint == IqopConstraintBlock::area_minus) &&
        variable == IqopVariableBlock::offsets)
    {
        const bool upper = constraint == IqopConstraintBlock::area_plus;
        const std::vector<double> gradient = area_model_gradient(decision, upper);
        for (std::size_t i = 0; i < count; ++i)
        {
            entries.push_back({0, i, upper ? gradient[i] : -gradient[i]});
        }
        return entries;
    }
    if (constraint == IqopConstraintBlock::area_plus && variable == IqopVariableBlock::area_plus)
    {
        return {{0, 0, -1.0}};
    }
    if (constraint == IqopConstraintBlock::area_minus && variable == IqopVariableBlock::area_minus)
    {
        return {{0, 0, 1.0}};
    }
    if (constraint == IqopConstraintBlock::iso_quotient)
    {
        const double area_to_perimeter_squared = geometry_.area().value() / (perimeter * perimeter);
        if (variable == IqopVariableBlock::length)
        {
            return {{0, 0, 2.0 * decision.length.value}};
        }
        if (variable == IqopVariableBlock::area_minus)
        {
            return {{0, 0, -FOUR_PI * area_to_perimeter_squared * decision.quotient.value}};
        }
        if (variable == IqopVariableBlock::quotient)
        {
            return {{0, 0, -FOUR_PI * area_to_perimeter_squared * decision.area_minus.value}};
        }
    }
    return entries;
}

double IqopSubproblem::objective(const IqopDecision &decision) const
{
    validate_decision(decision);
    double value = 0.0;
    if (options_.optimize_Q)
    {
        value += options_.lambda_Q * decision.quotient.value;
    }
    if (options_.optimize_S)
    {
        value += options_.lambda_S * decision.area_plus.value;
    }
    if (options_.optimize_L)
    {
        value += options_.lambda_L * decision.length.value;
    }
    return value;
}
} // namespace nepath
