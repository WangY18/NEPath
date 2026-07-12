#pragma once

#include <cstddef>

namespace nepath
{
struct ParentPathFrame;
struct OffsetPathFrame;
struct LengthDimension;
struct AreaDimension;
struct DimensionlessDimension;

template <typename Dimension, typename Frame> class IqopQuantity
{
  public:
    explicit constexpr IqopQuantity(double value) noexcept : value_(value) {}

    [[nodiscard]] constexpr double value() const noexcept
    {
        return value_;
    }

  private:
    double value_;
};

using ParentLength = IqopQuantity<LengthDimension, ParentPathFrame>;
using ParentArea = IqopQuantity<AreaDimension, ParentPathFrame>;
using ParentDirection = IqopQuantity<DimensionlessDimension, ParentPathFrame>;
using OffsetLength = IqopQuantity<LengthDimension, OffsetPathFrame>;
using OffsetArea = IqopQuantity<AreaDimension, OffsetPathFrame>;

struct ParentPoint2
{
    ParentLength x;
    ParentLength y;
};

struct ParentDirection2
{
    ParentDirection x;
    ParentDirection y;
};
} // namespace nepath
