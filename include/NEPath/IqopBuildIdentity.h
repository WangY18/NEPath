#pragma once

#include <string>

namespace nepath
{
class IqopSubproblem;

struct IqopBuildIdentity final
{
    std::string formulation_version;
    std::string solver_configuration_version;
    std::string ifopt_revision;
    std::string input_sha256;
};

[[nodiscard]] IqopBuildIdentity build_iqop_identity(const IqopSubproblem &subproblem);
} // namespace nepath
