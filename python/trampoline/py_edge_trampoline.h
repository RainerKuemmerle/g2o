#pragma once

#include "g2opy.h"

template <class EdgeBase>
class PyEdgeTrampoline : public EdgeBase,
                         public py::trampoline_self_life_support {
 public:
  using EdgeBase::EdgeBase;  // Inherit constructors
  void computeError() override {
    PYBIND11_OVERRIDE_NAME(void, EdgeBase, "compute_error", computeError, );
  }
  void linearizeOplus() override {
    PYBIND11_OVERRIDE_NAME(void, EdgeBase, "linearize_oplus", linearizeOplus, );
  }
};
