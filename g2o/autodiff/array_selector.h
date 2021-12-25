// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2020 Google Inc. All rights reserved.
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
// Author: darius.rueckert@fau.de (Darius Rueckert)
//

#ifndef G2O_CERES_PUBLIC_INTERNAL_ARRAY_SELECTOR_H_
#define G2O_CERES_PUBLIC_INTERNAL_ARRAY_SELECTOR_H_

#include <array>
#include <vector>

#include "fixed_array.h"
#include "types.h"

namespace g2o {
namespace ceres {
namespace internal {

// StaticFixedArray selects the best array implementation based on template
// arguments. If the size is not known at compile-time, pass
// ceres::DYNAMIC as a size-template argument.
//
// Three different containers are selected in different scenarios:
//
//   num_elements == DYNAMIC:
//      -> ceres::internal::FixedArray<T, max_stack_size>(size)

//   num_elements != DYNAMIC  &&  num_elements <= max_stack_size
//      -> std::array<T,num_elements>

//   num_elements != DYNAMIC  &&  num_elements >  max_stack_size
//      -> std::vector<T>(num_elements)
//
template <typename T, int NumElements, int MaxNumElementsOnStack,
          bool Dynamic = (NumElements == kDynamic),
          bool FitsOnStack = (NumElements <= MaxNumElementsOnStack)>
struct ArraySelector {};

template <typename T, int NumElements, int MaxNumElementsOnStack,
          bool FitsOnStack>
struct ArraySelector<T, NumElements, MaxNumElementsOnStack, true, FitsOnStack>
    : ceres::internal::FixedArray<T, MaxNumElementsOnStack> {
  explicit ArraySelector(int s)
      : ceres::internal::FixedArray<T, MaxNumElementsOnStack>(s) {}
};

template <typename T, int NumElements, int MaxNumElementsOnStack>
struct ArraySelector<T, NumElements, MaxNumElementsOnStack, false, true>
    : std::array<T, NumElements> {
  explicit ArraySelector(int /*s*/) {}
};

template <typename T, int NumElements, int MaxNumElementsOnStack>
struct ArraySelector<T, NumElements, MaxNumElementsOnStack, false, false>
    : std::vector<T> {
  explicit ArraySelector(int s) : std::vector<T>(s) {}
};

}  // namespace internal
}  // namespace ceres
}  // namespace g2o

#endif  // G2O_CERES_PUBLIC_INTERNAL_ARRAY_SELECTOR_H_
