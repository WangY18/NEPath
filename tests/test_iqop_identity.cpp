#include <NEPath/IqopBuildIdentity.h>
#include <NEPath/IqopSubproblem.h>
#include <NEPath/path.h>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cctype>

namespace
{
nepath::path make_square()
{
    constexpr std::array<double, 4> x{0.0, 2.0, 2.0, 0.0};
    constexpr std::array<double, 4> y{0.0, 0.0, 2.0, 2.0};
    return nepath::path(x.data(), y.data(), static_cast<int>(x.size()));
}
} // namespace

TEST_CASE("IQOP build identity is deterministic and content addressed")
{
    const nepath::IqopGeometry geometry = nepath::IqopGeometry::build(make_square());
    nepath::NonEquidistantOptions options;
    options.optimize_Q = true;
    options.optimize_S = false;
    options.optimize_L = false;
    const nepath::IqopSubproblem first = nepath::IqopSubproblem::build(geometry, options);
    const nepath::IqopSubproblem same = nepath::IqopSubproblem::build(geometry, options);
    nepath::NormalizedIqopOffsets changed_reference = first.reference_offsets();
    changed_reference.values.at(0) += 0.01;
    const nepath::IqopSubproblem changed = nepath::IqopSubproblem::build(geometry, options, changed_reference);

    const nepath::IqopBuildIdentity first_identity = nepath::build_iqop_identity(first);
    const nepath::IqopBuildIdentity same_identity = nepath::build_iqop_identity(same);
    const nepath::IqopBuildIdentity changed_identity = nepath::build_iqop_identity(changed);

    REQUIRE(first_identity.input_sha256 == same_identity.input_sha256);
    REQUIRE(first_identity.input_sha256 != changed_identity.input_sha256);
    REQUIRE(first_identity.input_sha256.size() == 64);
    REQUIRE(std::all_of(first_identity.input_sha256.begin(), first_identity.input_sha256.end(),
                        [](unsigned char value) { return std::isdigit(value) != 0 || (value >= 'a' && value <= 'f'); }));
    REQUIRE(first_identity.formulation_version == "iqop-scp-v1");
    REQUIRE(first_identity.solver_configuration_version == "ipopt-lbfgs-v8");
    REQUIRE(first_identity.ifopt_revision != "unavailable");
}
