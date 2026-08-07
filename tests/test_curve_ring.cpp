#include <NEPath/Curve.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
constexpr double RING_PARAMETER_TOLERANCE = 64.0 * std::numeric_limits<double>::epsilon();

void require_close(double actual, double expected, const std::string &context)
{
    if (std::abs(actual - expected) > RING_PARAMETER_TOLERANCE)
    {
        throw std::runtime_error(context + ": unexpected ring parameter");
    }
}

template <typename ExpectedError, typename Operation> void require_throws(Operation operation, const std::string &context)
{
    try
    {
        operation();
    }
    catch (const ExpectedError &)
    {
        return;
    }
    throw std::runtime_error(context + ": expected exception was not raised");
}
} // namespace

int main()
{
    try
    {
        constexpr int LENGTH = 4;
        const double x[LENGTH] = {0.0, 1.0, 1.0, 0.0};
        const double y[LENGTH] = {0.0, 0.0, 1.0, 1.0};
        constexpr double PERIMETER = 4.0;
        constexpr double HALF_EDGE = 0.5;
        constexpr double MANY_REVOLUTIONS = 100.0;

        require_close(nepath::Curve::interp_id(x, LENGTH, LENGTH), x[0], "periodic interpolation");
        require_close(nepath::Curve::ForDis(x, y, LENGTH, 0.0, MANY_REVOLUTIONS * PERIMETER + HALF_EDGE), 0.5, "forward distance modulo perimeter");
        require_close(nepath::Curve::BackDis(x, y, LENGTH, 0.0, MANY_REVOLUTIONS * PERIMETER + HALF_EDGE), 3.5, "backward distance modulo perimeter");
        require_close(nepath::Curve::ForDis(x, y, LENGTH, 2.25, 0.0), 2.25, "zero forward distance");
        require_close(nepath::Curve::BackDis(x, y, LENGTH, 2.25, 0.0), 2.25, "zero backward distance");

        const double degenerate[LENGTH] = {0.0, 0.0, 0.0, 0.0};
        require_throws<nepath::DegenerateCurveError>([&]() { nepath::Curve::ForDis(degenerate, degenerate, LENGTH, 0.0, HALF_EDGE); }, "degenerate curve");
        require_throws<nepath::InvalidCurveDistanceError>([&]() { nepath::Curve::ForDis(x, y, LENGTH, 0.0, std::numeric_limits<double>::infinity()); },
                                                          "infinite distance");
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
