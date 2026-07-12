#include <NEPath/IqopGeometry.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace
{
constexpr std::size_t MINIMUM_CLOSED_PATH_VERTICES = 3;

// Sixty-four ulps separates geometric degeneracy from one-step rounding noise.
constexpr double GEOMETRY_RELATIVE_FLOOR = 64.0 * std::numeric_limits<double>::epsilon();
} // namespace

namespace nepath
{
IqopGeometry::IqopGeometry(std::vector<ParentPoint2> points, std::vector<ParentDirection2> normals, std::vector<ParentLength> edge_lengths,
                           ParentArea area, ParentLength perimeter) noexcept
    : points_(std::move(points)), normals_(std::move(normals)), edge_lengths_(std::move(edge_lengths)), area_(area), perimeter_(perimeter)
{
}

IqopGeometry IqopGeometry::build(const path &input)
{
    if (input.length < static_cast<int>(MINIMUM_CLOSED_PATH_VERTICES))
    {
        throw InsufficientIqopVerticesError("IQOP requires at least three path vertices");
    }
    if (input.x == nullptr || input.y == nullptr)
    {
        throw NonFiniteIqopGeometryError("IQOP path coordinate storage is null");
    }

    const std::size_t vertex_count = static_cast<std::size_t>(input.length);
    std::vector<ParentPoint2> points;
    points.reserve(vertex_count);

    double x_min = input.x[0];
    double x_max = input.x[0];
    double y_min = input.y[0];
    double y_max = input.y[0];
    for (std::size_t i = 0; i < vertex_count; ++i)
    {
        if (!std::isfinite(input.x[i]) || !std::isfinite(input.y[i]))
        {
            std::ostringstream message;
            message << "IQOP path contains a non-finite coordinate at vertex " << i;
            throw NonFiniteIqopGeometryError(message.str());
        }
        points.push_back({ParentLength(input.x[i]), ParentLength(input.y[i])});
        x_min = std::min(x_min, input.x[i]);
        x_max = std::max(x_max, input.x[i]);
        y_min = std::min(y_min, input.y[i]);
        y_max = std::max(y_max, input.y[i]);
    }

    const double characteristic_length = std::hypot(x_max - x_min, y_max - y_min);
    if (!std::isfinite(characteristic_length) || characteristic_length == 0.0)
    {
        throw DegenerateIqopEdgeError("IQOP path has zero geometric extent");
    }
    const double length_floor = GEOMETRY_RELATIVE_FLOOR * characteristic_length;
    const double area_floor = GEOMETRY_RELATIVE_FLOOR * characteristic_length * characteristic_length;

    std::vector<ParentLength> edge_lengths;
    edge_lengths.reserve(vertex_count);
    double perimeter = 0.0;
    double twice_area = 0.0;
    for (std::size_t i = 0; i < vertex_count; ++i)
    {
        const std::size_t next = (i + 1) % vertex_count;
        const double dx = input.x[next] - input.x[i];
        const double dy = input.y[next] - input.y[i];
        const double edge_length = std::hypot(dx, dy);
        if (edge_length <= length_floor)
        {
            std::ostringstream message;
            message << "IQOP path has a degenerate edge at index " << i;
            throw DegenerateIqopEdgeError(message.str());
        }
        edge_lengths.emplace_back(edge_length);
        perimeter += edge_length;
        twice_area += input.x[i] * input.y[next] - input.y[i] * input.x[next];
    }

    const double area = 0.5 * twice_area;
    if (area <= area_floor)
    {
        throw NonPositiveIqopAreaError("IQOP requires a counter-clockwise path with positive area");
    }

    std::vector<ParentDirection2> normals;
    normals.reserve(vertex_count);
    for (std::size_t i = 0; i < vertex_count; ++i)
    {
        const std::size_t previous = (i + vertex_count - 1) % vertex_count;
        const std::size_t next = (i + 1) % vertex_count;
        const double tangent_x = input.x[next] - input.x[previous];
        const double tangent_y = input.y[next] - input.y[previous];
        const double tangent_length = std::hypot(tangent_x, tangent_y);
        if (tangent_length <= length_floor)
        {
            std::ostringstream message;
            message << "IQOP inward normal is undefined at vertex " << i;
            throw DegenerateIqopNormalError(message.str());
        }
        normals.push_back({ParentDirection(-tangent_y / tangent_length), ParentDirection(tangent_x / tangent_length)});
    }

    return IqopGeometry(std::move(points), std::move(normals), std::move(edge_lengths), ParentArea(area), ParentLength(perimeter));
}
} // namespace nepath
