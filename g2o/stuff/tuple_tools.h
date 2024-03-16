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
#include <type_traits>
#include <utility>

namespace g2o {

namespace internal {
template <std::size_t... Ns, typename... Ts>
auto tail_impl(std::index_sequence<Ns...>, std::tuple<Ts...> t) {
  return std::make_tuple(std::get<Ns + 1U>(t)...);
}

template <typename F, typename T, std::size_t... I>
void tuple_apply_i_impl(F&& f, T& t, int i, std::index_sequence<I...>) {
  (..., (I == i ? f(std::get<I>(t)) : void()));
}
}  // namespace internal

template <typename F, typename T>
void tuple_apply_i(F&& f, T& t, int i) {
  internal::tuple_apply_i_impl(
      f, t, i, std::make_index_sequence<std::tuple_size_v<std::decay_t<T>>>());
}

template <typename T, typename... Ts>
auto tupple_head(std::tuple<T, Ts...> t) {
  return std::get<0>(t);
}

template <typename... Ts>
auto tuple_tail(std::tuple<Ts...> t) {
  return internal::tail_impl(std::make_index_sequence<sizeof...(Ts) - 1U>(), t);
}

}  // namespace g2o
