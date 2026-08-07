#include "cfs_binding.h"

#include <NEPath/Connector.h>
#include <NEPath/Curve.h>
#include <NEPath/path.h>

#include <nanobind/stl/vector.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>

namespace nb = nanobind;
using namespace nb::literals;

namespace
{
constexpr int MINIMUM_CONTOUR_VERTICES = 3;

// Matches NEPathPlanner's default sampling contract for stable CFS geometry.
constexpr int MINIMUM_CFS_SAMPLES_PER_CONTOUR = 50;

// Bounds quadratic simple-polygon validation to about 12.5 million edge pairs per contour.
constexpr int MAXIMUM_CFS_INPUT_VERTICES_PER_CONTOUR = 5'000;

// Bounds aggregate input copying and pairwise containment work across contour families.
constexpr std::size_t MAXIMUM_CFS_INPUT_VERTICES = 50'000;

// Caps self-intersection and cross-contour edge comparisons before quadratic geometry work begins.
constexpr std::size_t MAXIMUM_CFS_GEOMETRY_COMPARISONS = 50'000'000;

// Sixty-four ulps distinguish geometric degeneracy from one-step rounding noise.
constexpr double GEOMETRY_RELATIVE_FLOOR = 64.0 * std::numeric_limits<double>::epsilon();

// CFS internally samples at one fifth of the requested toolpath spacing.
constexpr double CFS_RESAMPLE_RATIO = 0.2;

// One million paired double coordinates consume 16 MB before connector working copies.
constexpr std::size_t MAXIMUM_CFS_WAYPOINTS = 1'000'000;

struct ValidatedContour final
{
    double absolute_area;
    double perimeter;
    double characteristic_length;
};

struct ContainmentTree final
{
    std::vector<std::size_t> parents;
    std::size_t root;
};

struct Point final
{
    double x;
    double y;
};

[[nodiscard]] double orientation(const Point &first, const Point &second, const Point &third)
{
    return (second.x - first.x) * (third.y - first.y) - (second.y - first.y) * (third.x - first.x);
}

[[nodiscard]] bool point_on_segment(const Point &point, const Point &first, const Point &second, double length_floor, double area_floor)
{
    if (std::abs(orientation(first, second, point)) > area_floor)
    {
        return false;
    }
    return point.x >= std::min(first.x, second.x) - length_floor && point.x <= std::max(first.x, second.x) + length_floor &&
           point.y >= std::min(first.y, second.y) - length_floor && point.y <= std::max(first.y, second.y) + length_floor;
}

[[nodiscard]] bool segments_intersect_or_touch(const Point &a, const Point &b, const Point &c, const Point &d, double characteristic_length)
{
    const double length_floor = GEOMETRY_RELATIVE_FLOOR * characteristic_length;
    const double area_floor = GEOMETRY_RELATIVE_FLOOR * characteristic_length * characteristic_length;
    if (std::max(a.x, b.x) + length_floor < std::min(c.x, d.x) || std::max(c.x, d.x) + length_floor < std::min(a.x, b.x) ||
        std::max(a.y, b.y) + length_floor < std::min(c.y, d.y) || std::max(c.y, d.y) + length_floor < std::min(a.y, b.y))
    {
        return false;
    }
    const double abc = orientation(a, b, c);
    const double abd = orientation(a, b, d);
    const double cda = orientation(c, d, a);
    const double cdb = orientation(c, d, b);

    const bool ab_straddles_cd = (abc > area_floor && abd < -area_floor) || (abc < -area_floor && abd > area_floor);
    const bool cd_straddles_ab = (cda > area_floor && cdb < -area_floor) || (cda < -area_floor && cdb > area_floor);
    if (ab_straddles_cd && cd_straddles_ab)
    {
        return true;
    }
    return point_on_segment(c, a, b, length_floor, area_floor) || point_on_segment(d, a, b, length_floor, area_floor) ||
           point_on_segment(a, c, d, length_floor, area_floor) || point_on_segment(b, c, d, length_floor, area_floor);
}

void validate_input_budget(const std::vector<nepath::path> &contours)
{
    std::size_t total_vertices = 0;
    std::size_t geometry_comparisons = 0;
    for (std::size_t index = 0; index < contours.size(); ++index)
    {
        const int length = contours[index].length;
        if (length > MAXIMUM_CFS_INPUT_VERTICES_PER_CONTOUR)
        {
            std::ostringstream message;
            message << "CFS contour " << index << " has " << length << " vertices; simplify it to at most " << MAXIMUM_CFS_INPUT_VERTICES_PER_CONTOUR
                    << " vertices before connection";
            throw nepath::python::InvalidCfsContourError(message.str());
        }
        if (length > 0)
        {
            const std::size_t vertices = static_cast<std::size_t>(length);
            const std::size_t self_comparisons = vertices >= MINIMUM_CONTOUR_VERTICES ? vertices * (vertices - 3) / 2 : 0;
            const std::size_t cross_comparisons = vertices * total_vertices;
            const std::size_t added_comparisons = self_comparisons + cross_comparisons;
            if (added_comparisons > MAXIMUM_CFS_GEOMETRY_COMPARISONS - geometry_comparisons)
            {
                std::ostringstream message;
                message << "CFS input requires more than " << MAXIMUM_CFS_GEOMETRY_COMPARISONS
                        << " boundary comparisons; simplify the contour family before connection";
                throw nepath::python::InvalidCfsTopologyError(message.str());
            }
            geometry_comparisons += added_comparisons;
            total_vertices += vertices;
        }
        if (total_vertices > MAXIMUM_CFS_INPUT_VERTICES)
        {
            std::ostringstream message;
            message << "CFS input has more than " << MAXIMUM_CFS_INPUT_VERTICES << " total vertices; simplify contours before connection";
            throw nepath::python::InvalidCfsTopologyError(message.str());
        }
    }
}

[[nodiscard]] bool boundaries_intersect_or_touch(const nepath::path &first, const nepath::path &second, double characteristic_length)
{
    for (int first_edge = 0; first_edge < first.length; ++first_edge)
    {
        const int first_next = (first_edge + 1) % first.length;
        const Point a{first.x[first_edge], first.y[first_edge]};
        const Point b{first.x[first_next], first.y[first_next]};
        for (int second_edge = 0; second_edge < second.length; ++second_edge)
        {
            const int second_next = (second_edge + 1) % second.length;
            const Point c{second.x[second_edge], second.y[second_edge]};
            const Point d{second.x[second_next], second.y[second_next]};
            if (segments_intersect_or_touch(a, b, c, d, characteristic_length))
            {
                return true;
            }
        }
    }
    return false;
}

void validate_simple_contour(const nepath::path &contour, std::size_t index, double characteristic_length)
{
    for (int first_edge = 0; first_edge < contour.length; ++first_edge)
    {
        const int first_next = (first_edge + 1) % contour.length;
        const Point a{contour.x[first_edge], contour.y[first_edge]};
        const Point b{contour.x[first_next], contour.y[first_next]};
        for (int second_edge = first_edge + 1; second_edge < contour.length; ++second_edge)
        {
            const int second_next = (second_edge + 1) % contour.length;
            const bool adjacent = first_next == second_edge || second_next == first_edge;
            if (adjacent)
            {
                continue;
            }
            const Point c{contour.x[second_edge], contour.y[second_edge]};
            const Point d{contour.x[second_next], contour.y[second_next]};
            if (segments_intersect_or_touch(a, b, c, d, characteristic_length))
            {
                std::ostringstream message;
                message << "CFS contour " << index << " has a self-intersection between edges " << first_edge << " and " << second_edge;
                throw nepath::python::InvalidCfsContourError(message.str());
            }
        }
    }
}

[[nodiscard]] bool point_in_polygon(double x, double y, const nepath::path &polygon)
{
    bool inside = false;
    for (int current = 0, previous = polygon.length - 1; current < polygon.length; previous = current++)
    {
        const double current_x = polygon.x[current];
        const double current_y = polygon.y[current];
        const double previous_x = polygon.x[previous];
        const double previous_y = polygon.y[previous];
        const bool crosses_scanline = (current_y > y) != (previous_y > y);
        if (crosses_scanline && x < (previous_x - current_x) * (y - current_y) / (previous_y - current_y) + current_x)
        {
            inside = !inside;
        }
    }
    return inside;
}

[[nodiscard]] bool contains_contour(const nepath::path &outer, const nepath::path &inner)
{
    for (int i = 0; i < inner.length; ++i)
    {
        if (!point_in_polygon(inner.x[i], inner.y[i], outer))
        {
            return false;
        }
    }
    return true;
}

[[nodiscard]] ValidatedContour validate_contour(const nepath::path &contour, std::size_t index)
{
    if (contour.length < MINIMUM_CONTOUR_VERTICES || contour.x == nullptr || contour.y == nullptr)
    {
        std::ostringstream message;
        message << "CFS contour " << index << " requires at least three stored vertices";
        throw nepath::python::InvalidCfsContourError(message.str());
    }

    double x_min = contour.x[0];
    double x_max = contour.x[0];
    double y_min = contour.y[0];
    double y_max = contour.y[0];
    for (int i = 0; i < contour.length; ++i)
    {
        if (!std::isfinite(contour.x[i]) || !std::isfinite(contour.y[i]))
        {
            std::ostringstream message;
            message << "CFS contour " << index << " contains a non-finite coordinate at vertex " << i;
            throw nepath::python::InvalidCfsContourError(message.str());
        }
        x_min = std::min(x_min, contour.x[i]);
        x_max = std::max(x_max, contour.x[i]);
        y_min = std::min(y_min, contour.y[i]);
        y_max = std::max(y_max, contour.y[i]);
    }

    const double characteristic_length = std::hypot(x_max - x_min, y_max - y_min);
    if (!std::isfinite(characteristic_length) || characteristic_length == 0.0)
    {
        std::ostringstream message;
        message << "CFS contour " << index << " has zero geometric extent";
        throw nepath::python::InvalidCfsContourError(message.str());
    }
    const double length_floor = GEOMETRY_RELATIVE_FLOOR * characteristic_length;
    const double area_floor = GEOMETRY_RELATIVE_FLOOR * characteristic_length * characteristic_length;

    double twice_area = 0.0;
    double perimeter = 0.0;
    for (int i = 0; i < contour.length; ++i)
    {
        const int next = (i + 1) % contour.length;
        const double edge_length = std::hypot(contour.x[next] - contour.x[i], contour.y[next] - contour.y[i]);
        if (edge_length <= length_floor)
        {
            std::ostringstream message;
            message << "CFS contour " << index << " has a degenerate edge at index " << i;
            throw nepath::python::InvalidCfsContourError(message.str());
        }
        perimeter += edge_length;
        twice_area += contour.x[i] * contour.y[next] - contour.y[i] * contour.x[next];
    }

    const double absolute_area = 0.5 * std::abs(twice_area);
    if (!std::isfinite(absolute_area) || absolute_area <= area_floor)
    {
        std::ostringstream message;
        message << "CFS contour " << index << " has zero enclosed area";
        throw nepath::python::InvalidCfsContourError(message.str());
    }
    validate_simple_contour(contour, index, characteristic_length);
    return ValidatedContour{absolute_area, perimeter, characteristic_length};
}

void validate_sampling_budget(const std::vector<ValidatedContour> &contours, double spacing)
{
    const long double sample_distance = static_cast<long double>(spacing) * CFS_RESAMPLE_RATIO;
    long double resampled_waypoints = 0.0L;
    long double total_perimeter = 0.0L;
    long double maximum_characteristic_length = 0.0L;
    for (const ValidatedContour &contour : contours)
    {
        resampled_waypoints +=
            std::max(std::ceil(static_cast<long double>(contour.perimeter) / sample_distance), static_cast<long double>(MINIMUM_CFS_SAMPLES_PER_CONTOUR));
        total_perimeter += contour.perimeter;
        maximum_characteristic_length = std::max(maximum_characteristic_length, static_cast<long double>(contour.characteristic_length));
    }
    const long double maximum_tree_transitions = 2.0L * static_cast<long double>(contours.size() - 1);
    const long double connected_length_bound = total_perimeter + maximum_tree_transitions * maximum_characteristic_length;
    const long double connected_waypoints = std::ceil(connected_length_bound / sample_distance);
    const long double estimated_waypoints = std::max(resampled_waypoints, connected_waypoints);
    if (!std::isfinite(estimated_waypoints) || estimated_waypoints > MAXIMUM_CFS_WAYPOINTS)
    {
        std::ostringstream message;
        message << "CFS spacing would create approximately " << estimated_waypoints << " sampled waypoints; increase spacing to stay within the "
                << MAXIMUM_CFS_WAYPOINTS << " waypoint safety limit";
        throw nepath::python::InvalidCfsSpacingError(message.str());
    }
}

[[nodiscard]] ContainmentTree build_containment_tree(const std::vector<nepath::path> &contours, const std::vector<ValidatedContour> &validated)
{
    const std::size_t count = contours.size();
    const std::size_t no_parent = count;
    std::vector<std::size_t> parents(count, no_parent);
    std::size_t root = no_parent;
    std::size_t root_count = 0;

    for (std::size_t first = 0; first < count; ++first)
    {
        for (std::size_t second = first + 1; second < count; ++second)
        {
            const double characteristic_length = std::max(validated[first].characteristic_length, validated[second].characteristic_length);
            if (boundaries_intersect_or_touch(contours[first], contours[second], characteristic_length))
            {
                std::ostringstream message;
                message << "CFS contours " << first << " and " << second << " intersect or touch";
                throw nepath::python::InvalidCfsTopologyError(message.str());
            }
        }
    }

    for (std::size_t child = 0; child < count; ++child)
    {
        double parent_area = std::numeric_limits<double>::infinity();
        for (std::size_t candidate = 0; candidate < count; ++candidate)
        {
            if (candidate == child || validated[candidate].absolute_area <= validated[child].absolute_area)
            {
                continue;
            }
            if (validated[candidate].absolute_area < parent_area && contains_contour(contours[candidate], contours[child]))
            {
                parents[child] = candidate;
                parent_area = validated[candidate].absolute_area;
            }
        }
        if (parents[child] == no_parent)
        {
            root = child;
            ++root_count;
        }
    }

    if (root_count != 1)
    {
        std::ostringstream message;
        message << "CFS contours must form one containment tree; found " << root_count << " outer roots";
        throw nepath::python::InvalidCfsTopologyError(message.str());
    }
    return ContainmentTree{std::move(parents), root};
}

[[nodiscard]] nepath::path connect_fermat_spiral(const std::vector<nepath::path> &contours, double spacing)
{
    if (contours.empty())
    {
        throw nepath::python::EmptyCfsContoursError("CFS requires at least one contour");
    }
    if (!std::isfinite(spacing) || spacing <= 0.0)
    {
        throw nepath::python::InvalidCfsSpacingError("CFS spacing must be finite and positive");
    }
    validate_input_budget(contours);

    std::vector<ValidatedContour> validated;
    validated.reserve(contours.size());
    for (std::size_t i = 0; i < contours.size(); ++i)
    {
        validated.push_back(validate_contour(contours[i], i));
    }
    validate_sampling_budget(validated, spacing);
    const ContainmentTree tree = build_containment_tree(contours, validated);

    std::vector<std::unique_ptr<nepath::pathnode>> owners;
    std::vector<nepath::pathnode *> nodes;
    owners.reserve(contours.size());
    nodes.reserve(contours.size());
    for (const nepath::path &contour : contours)
    {
        nepath::path sampled = nepath::Curve::wash_dis(contour, spacing * CFS_RESAMPLE_RATIO, MINIMUM_CFS_SAMPLES_PER_CONTOUR);
        auto node = std::make_unique<nepath::pathnode>();
        node->data.steal(sampled);
        owners.push_back(std::move(node));
        nodes.push_back(owners.back().get());
    }
    for (std::size_t child = 0; child < contours.size(); ++child)
    {
        if (child == tree.root)
        {
            continue;
        }
        const std::size_t parent = tree.parents[child];
        nodes[child]->parent = nodes[parent];
        nodes[parent]->children.push_back(nodes[child]);
    }

    nepath::pathnode *root = owners[tree.root].get();
    const nepath::Connector::NodeDeletionObserver release_deleted_owner = [&owners](nepath::pathnode *deleted)
    {
        for (std::unique_ptr<nepath::pathnode> &owner : owners)
        {
            if (owner.get() == deleted)
            {
                owner.release();
                return;
            }
        }
    };
    const nepath::path result = nepath::Connector::ConnectedFermatSpiral_MultMinimum(root, spacing, release_deleted_owner);
    return result;
}
} // namespace

namespace nepath::python
{
void register_cfs_binding(nb::module_ &module)
{
    nb::exception<CfsBindingError> base_error(module, "CfsBindingError", PyExc_RuntimeError);
    nb::exception<EmptyCfsContoursError>(module, "EmptyCfsContoursError", base_error);
    nb::exception<InvalidCfsSpacingError>(module, "InvalidCfsSpacingError", base_error);
    nb::exception<InvalidCfsContourError>(module, "InvalidCfsContourError", base_error);
    nb::exception<InvalidCfsTopologyError>(module, "InvalidCfsTopologyError", base_error);
    module.def("connect_fermat_spiral", &connect_fermat_spiral, "contours"_a, "spacing"_a,
               "Connect one validated containment tree of contours with NEPath's Connected Fermat Spiral algorithm.");
}
} // namespace nepath::python
