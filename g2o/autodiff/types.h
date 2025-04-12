// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2019 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: sameeragarwal@google.com (Sameer Agarwal)
//
// Enums and other top level class definitions.
//
// Note: internal/types.cc defines stringification routines for some
// of these enums. Please update those routines if you extend or
// remove enums from here.

#ifndef G2O_CERES_PUBLIC_TYPES_H_
#define G2O_CERES_PUBLIC_TYPES_H_

#include <string>

#include "disable_warnings.h"

namespace g2o {
namespace ceres {

// For SizedCostFunction and AutoDiffCostFunction, DYNAMIC can be
// specified for the number of residuals. If specified, then the
// number of residuas for that cost function can vary at runtime.
enum DimensionType {
  kDynamic = -1,
};

// It is a near impossibility that user code generates this exact
// value in normal operation, thus we will use it to fill arrays
// before passing them to user code. If on return an element of the
// array still contains this value, we will assume that the user code
// did not write to that memory location.
const double kImpossibleValue = 1e302;

}  // namespace ceres
}  // namespace g2o

#include "reenable_warnings.h"

#endif  // G2O_CERES_PUBLIC_TYPES_H_
