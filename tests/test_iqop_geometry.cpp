#include <NEPath/IqopGeometry.h>
#include <NEPath/path.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>

namespace
{
nepath::path make_square()
{
    constexpr std::array<double, 4> x{0.0, 2.0, 2.0, 0.0};
    constexpr std::array<double, 4> y{0.0, 0.0, 2.0, 2.0};
    return nepath::path(x.data(), y.data(), static_cast<int>(x.size()));
}
} // namespace

TEST_CASE("IQOP geometry owns validated cyclic path data")
{
    const nepath::path input = make_square();

    const nepath::IqopGeometry geometry = nepath::IqopGeometry::build(input);

    REQUIRE(geometry.vertex_count() == 4);
    REQUIRE(geometry.area().value() == Catch::Approx(4.0));
    REQUIRE(geometry.perimeter().value() == Catch::Approx(8.0));
    for (std::size_t i = 0; i < geometry.vertex_count(); ++i)
    {
        REQUIRE(geometry.edge_length(i).value() == Catch::Approx(2.0));
        REQUIRE(std::isfinite(geometry.normal_x(i).value()));
        REQUIRE(std::isfinite(geometry.normal_y(i).value()));
    }
}

TEST_CASE("IQOP geometry rejects fewer than three vertices")
{
    constexpr std::array<double, 2> x{0.0, 1.0};
    constexpr std::array<double, 2> y{0.0, 0.0};
    const nepath::path input(x.data(), y.data(), static_cast<int>(x.size()));

    REQUIRE_THROWS_AS(nepath::IqopGeometry::build(input), nepath::InsufficientIqopVerticesError);
}

TEST_CASE("IQOP geometry rejects a zero-length edge")
{
    constexpr std::array<double, 4> x{0.0, 2.0, 2.0, 2.0};
    constexpr std::array<double, 4> y{0.0, 0.0, 0.0, 2.0};
    const nepath::path input(x.data(), y.data(), static_cast<int>(x.size()));

    REQUIRE_THROWS_AS(nepath::IqopGeometry::build(input), nepath::DegenerateIqopEdgeError);
}

TEST_CASE("IQOP geometry rejects clockwise input")
{
    constexpr std::array<double, 4> x{0.0, 0.0, 2.0, 2.0};
    constexpr std::array<double, 4> y{0.0, 2.0, 2.0, 0.0};
    const nepath::path input(x.data(), y.data(), static_cast<int>(x.size()));

    REQUIRE_THROWS_AS(nepath::IqopGeometry::build(input), nepath::NonPositiveIqopAreaError);
}
