#ifndef G2O_PY_H
#define G2O_PY_H

// IWYU pragma: begin_exports
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/operators.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/unordered_set.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>

#include <memory>
#include <string>
#include <typeinfo>

namespace py = nanobind;             // NOLINT
using namespace nanobind::literals;  // NOLINT

// IWYU pragma: end_exports

#endif
