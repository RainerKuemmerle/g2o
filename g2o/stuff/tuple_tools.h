// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <tuple>
namespace g2o {

template <int I>
struct Tuple_apply_i {
  template <typename F, typename T>
  void operator()(F&& f, T& t, int i) {
    if (i == I - 1)
      f(std::get<I - 1>(t));
    else
      Tuple_apply_i<I - 1>()(f, t, i);
  }
};

template <>
struct Tuple_apply_i<0> {
  template <typename F, typename T>
  void operator()(F&&, T&, int) {}
};

template <typename F, typename T>
void tuple_apply_i(F&& f, T& t, int i) {
  Tuple_apply_i<std::tuple_size<T>::value>()(f, t, i);
}

template <typename Value, typename... Ts2>
std::tuple<Ts2...> tuple_init(const Value& value, const std::tuple<Ts2...>&) {
  return std::tuple<Ts2...>{Ts2{value}...};
}

}  // namespace g2o
