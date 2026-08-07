#pragma once

#include <nanobind/nanobind.h>

#include <stdexcept>
#include <string>

namespace nepath::python
{
class CfsBindingError : public std::runtime_error
{
public:
    explicit CfsBindingError(const std::string &message) : std::runtime_error(message) {}
};

class EmptyCfsContoursError final : public CfsBindingError
{
public:
    explicit EmptyCfsContoursError(const std::string &message) : CfsBindingError(message) {}
};

class InvalidCfsSpacingError final : public CfsBindingError
{
public:
    explicit InvalidCfsSpacingError(const std::string &message) : CfsBindingError(message) {}
};

class InvalidCfsContourError final : public CfsBindingError
{
public:
    explicit InvalidCfsContourError(const std::string &message) : CfsBindingError(message) {}
};

class InvalidCfsTopologyError final : public CfsBindingError
{
public:
    explicit InvalidCfsTopologyError(const std::string &message) : CfsBindingError(message) {}
};

void register_cfs_binding(nanobind::module_ &module);
} // namespace nepath::python
