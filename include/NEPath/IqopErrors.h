#pragma once

#include <stdexcept>
#include <string>

namespace nepath
{
class IqopGeometryError : public std::runtime_error
{
  public:
    explicit IqopGeometryError(const std::string &message) : std::runtime_error(message) {}
};

class InsufficientIqopVerticesError final : public IqopGeometryError
{
  public:
    explicit InsufficientIqopVerticesError(const std::string &message) : IqopGeometryError(message) {}
};

class NonFiniteIqopGeometryError final : public IqopGeometryError
{
  public:
    explicit NonFiniteIqopGeometryError(const std::string &message) : IqopGeometryError(message) {}
};

class DegenerateIqopEdgeError final : public IqopGeometryError
{
  public:
    explicit DegenerateIqopEdgeError(const std::string &message) : IqopGeometryError(message) {}
};

class DegenerateIqopNormalError final : public IqopGeometryError
{
  public:
    explicit DegenerateIqopNormalError(const std::string &message) : IqopGeometryError(message) {}
};

class NonPositiveIqopAreaError final : public IqopGeometryError
{
  public:
    explicit NonPositiveIqopAreaError(const std::string &message) : IqopGeometryError(message) {}
};

class IqopModelError : public std::runtime_error
{
  public:
    explicit IqopModelError(const std::string &message) : std::runtime_error(message) {}
};

class InvalidIqopOptionsError final : public IqopModelError
{
  public:
    explicit InvalidIqopOptionsError(const std::string &message) : IqopModelError(message) {}
};

class InvalidIqopDecisionError final : public IqopModelError
{
  public:
    explicit InvalidIqopDecisionError(const std::string &message) : IqopModelError(message) {}
};

class InvalidIqopSolverSettingsError final : public IqopModelError
{
  public:
    explicit InvalidIqopSolverSettingsError(const std::string &message) : IqopModelError(message) {}
};

class IqopSolveError : public IqopModelError
{
  public:
    explicit IqopSolveError(const std::string &message) : IqopModelError(message) {}
};

class IqopSolveTimeLimitError final : public IqopSolveError
{
  public:
    explicit IqopSolveTimeLimitError(const std::string &message) : IqopSolveError(message) {}
};
} // namespace nepath
