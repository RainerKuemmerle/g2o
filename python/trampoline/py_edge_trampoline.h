#pragma once

#include "g2opy.h"

template <class EdgeBase>
class PyEdgeTrampoline : public EdgeBase {
 public:
  NB_TRAMPOLINE(EdgeBase, 2);
  void computeError() override {
    NB_OVERRIDE_NAME("compute_error", computeError, );
  }
  void linearizeOplus() override {
    NB_OVERRIDE_NAME("linearize_oplus", linearizeOplus, );
  }
};
