#pragma once

#include <NEPath/IqopGeometry.h>
#include <NEPath/PlanningOptions.h>

#include <cstddef>
#include <vector>

namespace nepath
{
struct NormalizedIqopOffsets
{
    std::vector<double> values;
};

struct NormalizedIqopEdgeX
{
    std::vector<double> values;
};

struct NormalizedIqopEdgeY
{
    std::vector<double> values;
};

struct NormalizedIqopEdgeLength
{
    std::vector<double> values;
};

struct NormalizedIqopArea
{
    double value = 0.0;
};

struct NormalizedIqopLength
{
    double value = 0.0;
};

struct IsoperimetricQuotient
{
    double value = 1.0;
};

struct IqopDecision
{
    NormalizedIqopOffsets offsets;
    NormalizedIqopEdgeX edge_x;
    NormalizedIqopEdgeY edge_y;
    NormalizedIqopEdgeLength edge_length;
    NormalizedIqopArea area_plus;
    NormalizedIqopArea area_minus;
    NormalizedIqopLength length;
    IsoperimetricQuotient quotient;
};

enum class IqopVariableBlock
{
    offsets,
    edge_x,
    edge_y,
    edge_length,
    area_plus,
    area_minus,
    length,
    quotient
};

enum class IqopConstraintBlock
{
    smoothness,
    edge_decomposition,
    edge_norm,
    length_sum,
    area_plus,
    area_minus,
    iso_quotient
};

struct IqopJacobianEntry
{
    std::size_t row;
    std::size_t column;
    double value;
};

class IqopSubproblem final
{
  public:
    [[nodiscard]] static IqopSubproblem build(const IqopGeometry &geometry, const NonEquidistantOptions &options);
    [[nodiscard]] static IqopSubproblem build(const IqopGeometry &geometry, const NonEquidistantOptions &options,
                                              NormalizedIqopOffsets reference_offsets);

    [[nodiscard]] const IqopGeometry &geometry() const noexcept
    {
        return geometry_;
    }
    [[nodiscard]] const NonEquidistantOptions &options() const noexcept
    {
        return options_;
    }
    [[nodiscard]] const NormalizedIqopOffsets &reference_offsets() const noexcept
    {
        return reference_offsets_;
    }
    [[nodiscard]] const IqopDecision &initial_decision() const noexcept
    {
        return initial_decision_;
    }
    [[nodiscard]] const std::vector<IqopConstraintBlock> &active_constraints() const noexcept
    {
        return active_constraints_;
    }

    [[nodiscard]] bool has_edges() const noexcept
    {
        return options_.optimize_Q || options_.optimize_L;
    }
    [[nodiscard]] bool has_area_plus() const noexcept
    {
        return options_.optimize_S;
    }
    [[nodiscard]] bool has_area_minus() const noexcept
    {
        return options_.optimize_Q;
    }
    [[nodiscard]] bool has_length() const noexcept
    {
        return options_.optimize_Q || options_.optimize_L;
    }
    [[nodiscard]] bool has_quotient() const noexcept
    {
        return options_.optimize_Q;
    }

    [[nodiscard]] std::vector<double> evaluate(IqopConstraintBlock block, const IqopDecision &decision) const;
    [[nodiscard]] std::vector<IqopJacobianEntry> jacobian(IqopConstraintBlock constraint, IqopVariableBlock variable,
                                                          const IqopDecision &decision) const;
    [[nodiscard]] double true_normalized_area(const IqopDecision &decision) const;
    [[nodiscard]] double area_upper_model(const IqopDecision &decision) const;
    [[nodiscard]] double area_lower_model(const IqopDecision &decision) const;
    [[nodiscard]] double objective(const IqopDecision &decision) const;

  private:
    IqopSubproblem(IqopGeometry geometry, NonEquidistantOptions options, NormalizedIqopOffsets reference_offsets,
                   std::vector<double> area_linear_coefficients, std::vector<double> area_normal_coefficients,
                   IqopDecision initial_decision, std::vector<IqopConstraintBlock> active_constraints) noexcept;

    [[nodiscard]] std::vector<double> area_model_gradient(const IqopDecision &decision, bool upper) const;
    void validate_decision(const IqopDecision &decision) const;

    IqopGeometry geometry_;
    NonEquidistantOptions options_;
    NormalizedIqopOffsets reference_offsets_;
    std::vector<double> area_linear_coefficients_;
    std::vector<double> area_normal_coefficients_;
    IqopDecision initial_decision_;
    std::vector<IqopConstraintBlock> active_constraints_;
};
} // namespace nepath
