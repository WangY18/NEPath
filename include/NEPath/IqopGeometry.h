#pragma once

#include <NEPath/IqopErrors.h>
#include <NEPath/IqopTypes.h>
#include <NEPath/path.h>

#include <cstddef>
#include <vector>

namespace nepath
{
class IqopGeometry final
{
  public:
    [[nodiscard]] static IqopGeometry build(const path &input);

    [[nodiscard]] std::size_t vertex_count() const noexcept
    {
        return points_.size();
    }
    [[nodiscard]] const ParentPoint2 &point(std::size_t index) const
    {
        return points_.at(index);
    }
    [[nodiscard]] ParentDirection normal_x(std::size_t index) const
    {
        return normals_.at(index).x;
    }
    [[nodiscard]] ParentDirection normal_y(std::size_t index) const
    {
        return normals_.at(index).y;
    }
    [[nodiscard]] ParentLength edge_length(std::size_t index) const
    {
        return edge_lengths_.at(index);
    }
    [[nodiscard]] ParentArea area() const noexcept
    {
        return area_;
    }
    [[nodiscard]] ParentLength perimeter() const noexcept
    {
        return perimeter_;
    }

  private:
    IqopGeometry(std::vector<ParentPoint2> points, std::vector<ParentDirection2> normals, std::vector<ParentLength> edge_lengths,
                 ParentArea area, ParentLength perimeter) noexcept;

    std::vector<ParentPoint2> points_;
    std::vector<ParentDirection2> normals_;
    std::vector<ParentLength> edge_lengths_;
    ParentArea area_;
    ParentLength perimeter_;
};
} // namespace nepath
