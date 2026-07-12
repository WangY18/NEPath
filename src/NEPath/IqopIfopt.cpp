#include <NEPath/IqopIfopt.h>
#include <NEPath/IqopSolverDefaults.h>

#if defined(IncludeIpopt) && (IncludeIpopt != 0)
#include <IpIpoptApplication.hpp>
#include <IpReturnCodes.hpp>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/ipopt_adapter.h>
#include <ifopt/problem.h>
#include <ifopt/variable_set.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
#if defined(IncludeIpopt) && (IncludeIpopt != 0)
constexpr const char *OFFSETS_NAME = "deltas";
constexpr const char *EDGES_NAME = "edge_epigraph";
constexpr const char *SCALARS_NAME = "scalar_epigraph";

struct ScalarLayout final
{
    int area_plus = -1;
    int area_minus = -1;
    int length = -1;
    int quotient = -1;
    int count = 0;

    [[nodiscard]] static ScalarLayout build(const nepath::IqopSubproblem &subproblem)
    {
        ScalarLayout layout;
        if (subproblem.has_area_plus())
        {
            layout.area_plus = layout.count++;
        }
        if (subproblem.has_area_minus())
        {
            layout.area_minus = layout.count++;
        }
        if (subproblem.has_length())
        {
            layout.length = layout.count++;
        }
        if (subproblem.has_quotient())
        {
            layout.quotient = layout.count++;
        }
        return layout;
    }
};

class OffsetVariableSet final : public ifopt::VariableSet
{
  public:
    OffsetVariableSet(const nepath::IqopSubproblem &subproblem, const nepath::IqopDecision &initial)
        : VariableSet(static_cast<int>(subproblem.geometry().vertex_count()), OFFSETS_NAME),
          values_(static_cast<Eigen::Index>(subproblem.geometry().vertex_count())), lower_bound_(subproblem.options().alpha)
    {
        for (Eigen::Index i = 0; i < values_.size(); ++i)
        {
            values_[i] = initial.offsets.values.at(static_cast<std::size_t>(i));
        }
    }

    void SetVariables(const VectorXd &values) override
    {
        values_ = values;
    }
    [[nodiscard]] VectorXd GetValues() const override
    {
        return values_;
    }

    [[nodiscard]] VecBound GetBounds() const override
    {
        return VecBound(static_cast<std::size_t>(GetRows()), ifopt::Bounds(lower_bound_, 1.0));
    }

  private:
    VectorXd values_;
    double lower_bound_;
};

class EdgeVariableSet final : public ifopt::VariableSet
{
  public:
    EdgeVariableSet(const nepath::IqopSubproblem &subproblem, const nepath::IqopDecision &initial)
        : VariableSet(static_cast<int>(3 * subproblem.geometry().vertex_count()), EDGES_NAME),
          values_(static_cast<Eigen::Index>(3 * subproblem.geometry().vertex_count()))
    {
        const std::size_t count = subproblem.geometry().vertex_count();
        for (std::size_t i = 0; i < count; ++i)
        {
            values_[static_cast<Eigen::Index>(i)] = initial.edge_x.values.at(i);
            values_[static_cast<Eigen::Index>(count + i)] = initial.edge_y.values.at(i);
            values_[static_cast<Eigen::Index>(2 * count + i)] = initial.edge_length.values.at(i);
        }
    }

    void SetVariables(const VectorXd &values) override
    {
        values_ = values;
    }
    [[nodiscard]] VectorXd GetValues() const override
    {
        return values_;
    }

    [[nodiscard]] VecBound GetBounds() const override
    {
        return VecBound(static_cast<std::size_t>(GetRows()), ifopt::BoundGreaterZero);
    }

  private:
    VectorXd values_;
};

class ScalarVariableSet final : public ifopt::VariableSet
{
  public:
    ScalarVariableSet(const nepath::IqopSubproblem &subproblem, const nepath::IqopDecision &initial, ScalarLayout layout)
        : VariableSet(layout.count, SCALARS_NAME), values_(layout.count), layout_(layout)
    {
        if (layout_.area_plus >= 0)
        {
            values_[layout_.area_plus] = initial.area_plus.value;
        }
        if (layout_.area_minus >= 0)
        {
            values_[layout_.area_minus] = initial.area_minus.value;
        }
        if (layout_.length >= 0)
        {
            values_[layout_.length] = initial.length.value;
        }
        if (layout_.quotient >= 0)
        {
            values_[layout_.quotient] = initial.quotient.value;
        }
        (void)subproblem;
    }

    void SetVariables(const VectorXd &values) override
    {
        values_ = values;
    }
    [[nodiscard]] VectorXd GetValues() const override
    {
        return values_;
    }

    [[nodiscard]] VecBound GetBounds() const override
    {
        VecBound bounds(static_cast<std::size_t>(GetRows()), ifopt::BoundGreaterZero);
        if (layout_.quotient >= 0)
        {
            bounds.at(static_cast<std::size_t>(layout_.quotient)) = ifopt::Bounds(1.0, ifopt::inf);
        }
        return bounds;
    }

  private:
    VectorXd values_;
    ScalarLayout layout_;
};

[[nodiscard]] nepath::IqopDecision read_decision(const ifopt::Composite::Ptr &variables, const nepath::IqopSubproblem &subproblem,
                                                 const ScalarLayout &layout)
{
    nepath::IqopDecision decision;
    const Eigen::VectorXd offsets = variables->GetComponent(OFFSETS_NAME)->GetValues();
    decision.offsets.values.assign(offsets.data(), offsets.data() + offsets.size());
    const std::size_t count = subproblem.geometry().vertex_count();
    if (subproblem.has_edges())
    {
        const Eigen::VectorXd edges = variables->GetComponent(EDGES_NAME)->GetValues();
        decision.edge_x.values.resize(count);
        decision.edge_y.values.resize(count);
        decision.edge_length.values.resize(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            decision.edge_x.values[i] = edges[static_cast<Eigen::Index>(i)];
            decision.edge_y.values[i] = edges[static_cast<Eigen::Index>(count + i)];
            decision.edge_length.values[i] = edges[static_cast<Eigen::Index>(2 * count + i)];
        }
    }
    const Eigen::VectorXd scalars = variables->GetComponent(SCALARS_NAME)->GetValues();
    if (layout.area_plus >= 0)
    {
        decision.area_plus.value = scalars[layout.area_plus];
    }
    if (layout.area_minus >= 0)
    {
        decision.area_minus.value = scalars[layout.area_minus];
    }
    if (layout.length >= 0)
    {
        decision.length.value = scalars[layout.length];
    }
    if (layout.quotient >= 0)
    {
        decision.quotient.value = scalars[layout.quotient];
    }
    return decision;
}

[[nodiscard]] int constraint_rows(nepath::IqopConstraintBlock block, std::size_t vertex_count)
{
    switch (block)
    {
    case nepath::IqopConstraintBlock::smoothness:
    case nepath::IqopConstraintBlock::edge_decomposition:
        return static_cast<int>(4 * vertex_count);
    case nepath::IqopConstraintBlock::edge_norm:
        return static_cast<int>(vertex_count);
    case nepath::IqopConstraintBlock::length_sum:
    case nepath::IqopConstraintBlock::area_plus:
    case nepath::IqopConstraintBlock::area_minus:
    case nepath::IqopConstraintBlock::iso_quotient:
        return 1;
    }
    throw nepath::InvalidIqopDecisionError("Unknown IQOP constraint block");
}

[[nodiscard]] const char *constraint_name(nepath::IqopConstraintBlock block)
{
    switch (block)
    {
    case nepath::IqopConstraintBlock::smoothness:
        return "smoothness";
    case nepath::IqopConstraintBlock::edge_decomposition:
        return "edge_decomposition";
    case nepath::IqopConstraintBlock::edge_norm:
        return "edge_norm";
    case nepath::IqopConstraintBlock::length_sum:
        return "length_sum";
    case nepath::IqopConstraintBlock::area_plus:
        return "area_plus";
    case nepath::IqopConstraintBlock::area_minus:
        return "area_minus";
    case nepath::IqopConstraintBlock::iso_quotient:
        return "iso_quotient";
    }
    throw nepath::InvalidIqopDecisionError("Unknown IQOP constraint block");
}

class BlockConstraintSet final : public ifopt::ConstraintSet
{
  public:
    BlockConstraintSet(std::shared_ptr<const nepath::IqopSubproblem> subproblem, nepath::IqopConstraintBlock block, ScalarLayout layout)
        : ConstraintSet(constraint_rows(block, subproblem->geometry().vertex_count()), constraint_name(block)),
          subproblem_(std::move(subproblem)), block_(block), layout_(layout)
    {
    }

    [[nodiscard]] VectorXd GetValues() const override
    {
        const nepath::IqopDecision decision = read_decision(GetVariables(), *subproblem_, layout_);
        const std::vector<double> values = subproblem_->evaluate(block_, decision);
        return Eigen::Map<const Eigen::VectorXd>(values.data(), static_cast<Eigen::Index>(values.size()));
    }

    [[nodiscard]] VecBound GetBounds() const override
    {
        return VecBound(static_cast<std::size_t>(GetRows()), ifopt::BoundSmallerZero);
    }

    void FillJacobianBlock(std::string variable_set, Jacobian &jacobian) const override
    {
        const nepath::IqopDecision decision = read_decision(GetVariables(), *subproblem_, layout_);
        if (variable_set == OFFSETS_NAME)
        {
            fill(nepath::IqopVariableBlock::offsets, 0, jacobian, decision);
        }
        else if (variable_set == EDGES_NAME)
        {
            const int count = static_cast<int>(subproblem_->geometry().vertex_count());
            fill(nepath::IqopVariableBlock::edge_x, 0, jacobian, decision);
            fill(nepath::IqopVariableBlock::edge_y, count, jacobian, decision);
            fill(nepath::IqopVariableBlock::edge_length, 2 * count, jacobian, decision);
        }
        else if (variable_set == SCALARS_NAME)
        {
            if (layout_.area_plus >= 0)
            {
                fill(nepath::IqopVariableBlock::area_plus, layout_.area_plus, jacobian, decision);
            }
            if (layout_.area_minus >= 0)
            {
                fill(nepath::IqopVariableBlock::area_minus, layout_.area_minus, jacobian, decision);
            }
            if (layout_.length >= 0)
            {
                fill(nepath::IqopVariableBlock::length, layout_.length, jacobian, decision);
            }
            if (layout_.quotient >= 0)
            {
                fill(nepath::IqopVariableBlock::quotient, layout_.quotient, jacobian, decision);
            }
        }
    }

  private:
    void fill(nepath::IqopVariableBlock variable, int column_offset, Jacobian &jacobian, const nepath::IqopDecision &decision) const
    {
        for (const nepath::IqopJacobianEntry &entry : subproblem_->jacobian(block_, variable, decision))
        {
            jacobian.coeffRef(static_cast<Eigen::Index>(entry.row),
                              static_cast<Eigen::Index>(column_offset + static_cast<int>(entry.column))) = entry.value;
        }
    }

    std::shared_ptr<const nepath::IqopSubproblem> subproblem_;
    nepath::IqopConstraintBlock block_;
    ScalarLayout layout_;
};

class LinearScalarCost final : public ifopt::CostTerm
{
  public:
    LinearScalarCost(std::string name, int scalar_index, double weight) : CostTerm(name), scalar_index_(scalar_index), weight_(weight) {}

    void FillJacobianBlock(std::string variable_set, Jacobian &jacobian) const override
    {
        if (variable_set == SCALARS_NAME)
        {
            jacobian.coeffRef(0, scalar_index_) = weight_;
        }
    }

  private:
    [[nodiscard]] double GetCost() const override
    {
        return weight_ * GetVariables()->GetComponent(SCALARS_NAME)->GetValues()[scalar_index_];
    }

    int scalar_index_;
    double weight_;
};

[[nodiscard]] double maximum_violation(const nepath::IqopSubproblem &subproblem, const nepath::IqopDecision &decision)
{
    double violation = 0.0;
    for (const nepath::IqopConstraintBlock block : subproblem.active_constraints())
    {
        for (const double value : subproblem.evaluate(block, decision))
        {
            violation = std::max(violation, value);
        }
    }
    for (const double offset : decision.offsets.values)
    {
        violation = std::max(violation, subproblem.options().alpha - offset);
        violation = std::max(violation, offset - 1.0);
    }
    return violation;
}

void project_lower_bound(double &value, double lower, double tolerance, const char *name)
{
    if (!std::isfinite(value))
    {
        throw nepath::IqopSolveError(std::string("Ipopt returned non-finite ") + name);
    }
    if (value >= lower)
    {
        return;
    }
    if (lower - value > tolerance)
    {
        std::ostringstream message;
        message << "Ipopt returned " << name << " below its bound by " << lower - value;
        throw nepath::IqopSolveError(message.str());
    }
    value = lower;
}

void project_upper_bound(double &value, double upper, double tolerance, const char *name)
{
    if (!std::isfinite(value))
    {
        throw nepath::IqopSolveError(std::string("Ipopt returned non-finite ") + name);
    }
    if (value <= upper)
    {
        return;
    }
    if (value - upper > tolerance)
    {
        std::ostringstream message;
        message << "Ipopt returned " << name << " above its bound by " << value - upper;
        throw nepath::IqopSolveError(message.str());
    }
    value = upper;
}

void project_simple_bounds(nepath::IqopDecision &decision, const nepath::IqopSubproblem &subproblem, double tolerance)
{
    for (double &offset : decision.offsets.values)
    {
        project_lower_bound(offset, subproblem.options().alpha, tolerance, "normalized offset");
        project_upper_bound(offset, 1.0, tolerance, "normalized offset");
    }
    for (double &value : decision.edge_x.values)
    {
        project_lower_bound(value, 0.0, tolerance, "edge-x epigraph");
    }
    for (double &value : decision.edge_y.values)
    {
        project_lower_bound(value, 0.0, tolerance, "edge-y epigraph");
    }
    for (double &value : decision.edge_length.values)
    {
        project_lower_bound(value, 0.0, tolerance, "edge-length epigraph");
    }
    if (subproblem.has_area_plus())
    {
        project_lower_bound(decision.area_plus.value, 0.0, tolerance, "upper-area epigraph");
    }
    if (subproblem.has_area_minus())
    {
        project_lower_bound(decision.area_minus.value, 0.0, tolerance, "lower-area epigraph");
    }
    if (subproblem.has_length())
    {
        project_lower_bound(decision.length.value, 0.0, tolerance, "length epigraph");
    }
    if (subproblem.has_quotient())
    {
        project_lower_bound(decision.quotient.value, 1.0, tolerance, "quotient epigraph");
    }
}

struct IpoptRunSummary final
{
    int status;
    double wall_time_seconds;
};

[[nodiscard]] IpoptRunSummary run_ipopt(ifopt::Problem &problem, const nepath::IqopSolverSettings &settings)
{
    Ipopt::SmartPtr<Ipopt::IpoptApplication> application = IpoptApplicationFactory();
    application->Options()->SetNumericValue("tol", settings.tolerance);
    application->Options()->SetNumericValue("acceptable_tol", nepath::IQOP_ACCEPTABLE_TOLERANCE_MULTIPLIER * settings.tolerance);
    application->Options()->SetNumericValue("acceptable_constr_viol_tol",
                                            nepath::IQOP_ACCEPTABLE_CONSTRAINT_MULTIPLIER * settings.tolerance);
    application->Options()->SetIntegerValue("acceptable_iter", nepath::IQOP_ACCEPTABLE_ITERATION_COUNT);
    application->Options()->SetNumericValue("constr_viol_tol", settings.tolerance);
    application->Options()->SetIntegerValue("max_iter", settings.maximum_iterations);
    application->Options()->SetNumericValue("max_wall_time", settings.wall_time_limit.count());
    application->Options()->SetNumericValue("bound_push", nepath::IQOP_INTERIOR_PUSH);
    application->Options()->SetNumericValue("bound_frac", nepath::IQOP_INTERIOR_FRACTION);
    application->Options()->SetNumericValue("slack_bound_push", nepath::IQOP_INTERIOR_PUSH);
    application->Options()->SetNumericValue("slack_bound_frac", nepath::IQOP_INTERIOR_FRACTION);
    application->Options()->SetIntegerValue("print_level", settings.verbose ? 5 : 0);
    application->Options()->SetStringValue("sb", settings.verbose ? "no" : "yes");
    application->Options()->SetStringValue("mu_strategy", "adaptive");
    application->Options()->SetStringValue("nlp_scaling_method", "gradient-based");
    application->Options()->SetNumericValue("nlp_scaling_max_gradient", 100.0);
    application->Options()->SetNumericValue("obj_scaling_factor", 1.0);
    application->Options()->SetStringValue("jacobian_approximation", "exact");
    application->Options()->SetStringValue("hessian_approximation", "limited-memory");

    const Ipopt::ApplicationReturnStatus initialization_status = application->Initialize();
    if (initialization_status != Ipopt::Solve_Succeeded)
    {
        std::ostringstream message;
        message << "Ipopt failed to initialize IQOP solve, status " << initialization_status;
        throw nepath::IqopSolveError(message.str());
    }
    Ipopt::SmartPtr<Ipopt::TNLP> adapter = new Ipopt::IpoptAdapter(problem, false);
    const Ipopt::ApplicationReturnStatus status = application->OptimizeTNLP(adapter);
    return IpoptRunSummary{static_cast<int>(status), application->Statistics()->TotalWallclockTime()};
}
#endif
} // namespace

namespace nepath
{
IqopSolverSettings::IqopSolverSettings(double tolerance_value, int maximum_iterations_value, IqopSolverWallTime wall_time_limit_value,
                                       bool verbose_value) noexcept
    : tolerance(tolerance_value), maximum_iterations(maximum_iterations_value), wall_time_limit(wall_time_limit_value),
      verbose(verbose_value)
{
}

IqopSolverSettings IqopSolverSettings::build(double tolerance, int maximum_iterations, bool verbose)
{
    return build_bounded(tolerance, maximum_iterations, IqopSolverWallTime(IQOP_INNER_WALL_TIME_SECONDS), verbose);
}

IqopSolverSettings IqopSolverSettings::build_bounded(double tolerance, int maximum_iterations, IqopSolverWallTime wall_time_limit,
                                                     bool verbose)
{
    if (!std::isfinite(tolerance) || tolerance <= 0.0)
    {
        throw InvalidIqopSolverSettingsError("IQOP solver tolerance must be finite and positive");
    }
    if (maximum_iterations <= 0)
    {
        throw InvalidIqopSolverSettingsError("IQOP solver iteration limit must be positive");
    }
    if (!std::isfinite(wall_time_limit.count()) || wall_time_limit <= IqopSolverWallTime::zero())
    {
        throw InvalidIqopSolverSettingsError("IQOP solver wall-time limit must be finite and positive");
    }
    return IqopSolverSettings(tolerance, maximum_iterations, wall_time_limit, verbose);
}

IqopSolveResult solve_iqop_subproblem(const IqopSubproblem &subproblem, const IqopSolverSettings &settings)
{
#if defined(IncludeIpopt) && (IncludeIpopt != 0)
    const auto shared_subproblem = std::make_shared<const IqopSubproblem>(subproblem);
    const ScalarLayout layout = ScalarLayout::build(subproblem);
    ifopt::Problem problem;
    problem.AddVariableSet(std::make_shared<OffsetVariableSet>(subproblem, subproblem.initial_decision()));
    if (subproblem.has_edges())
    {
        problem.AddVariableSet(std::make_shared<EdgeVariableSet>(subproblem, subproblem.initial_decision()));
    }
    problem.AddVariableSet(std::make_shared<ScalarVariableSet>(subproblem, subproblem.initial_decision(), layout));

    for (const IqopConstraintBlock block : subproblem.active_constraints())
    {
        problem.AddConstraintSet(std::make_shared<BlockConstraintSet>(shared_subproblem, block, layout));
    }
    if (subproblem.options().optimize_Q)
    {
        problem.AddCostSet(std::make_shared<LinearScalarCost>("quotient_cost", layout.quotient, subproblem.options().lambda_Q));
    }
    if (subproblem.options().optimize_S)
    {
        problem.AddCostSet(std::make_shared<LinearScalarCost>("area_cost", layout.area_plus, subproblem.options().lambda_S));
    }
    if (subproblem.options().optimize_L)
    {
        problem.AddCostSet(std::make_shared<LinearScalarCost>("length_cost", layout.length, subproblem.options().lambda_L));
    }

    const IpoptRunSummary run = run_ipopt(problem, settings);
    const int status = run.status;
    if (status != Ipopt::Solve_Succeeded && status != Ipopt::Solved_To_Acceptable_Level)
    {
        const IqopDecision failed_decision = read_decision(problem.GetOptVariables(), subproblem, layout);
        std::ostringstream message;
        message << "Ipopt failed to solve IQOP subproblem, status " << status << ", iterations " << problem.GetIterationCount()
                << ", maximum constraint violation " << maximum_violation(subproblem, failed_decision) << ", vertices "
                << subproblem.geometry().vertex_count() << ", input " << build_iqop_identity(subproblem).input_sha256;
        if (status == Ipopt::Maximum_WallTime_Exceeded || status == Ipopt::Maximum_CpuTime_Exceeded)
        {
            throw IqopSolveTimeLimitError(message.str());
        }
        throw IqopSolveError(message.str());
    }

    IqopDecision decision = read_decision(problem.GetOptVariables(), subproblem, layout);
    project_simple_bounds(decision, subproblem, settings.tolerance);
    const IqopTerminationStatus termination_status =
        status == Ipopt::Solve_Succeeded ? IqopTerminationStatus::desired : IqopTerminationStatus::acceptable;
    return IqopSolveResult{decision,
                           subproblem.objective(decision),
                           maximum_violation(subproblem, decision),
                           problem.GetIterationCount(),
                           run.wall_time_seconds,
                           termination_status,
                           build_iqop_identity(subproblem)};
#else
    (void)subproblem;
    (void)settings;
    throw IqopSolveError("IQOP ifopt solve requested without Ipopt support");
#endif
}
} // namespace nepath
