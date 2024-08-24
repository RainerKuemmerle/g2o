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

#include "io_json.h"

#include <exception>
#include <iomanip>
#include <optional>

#include "g2o/config.h"
#include "g2o/core/abstract_graph.h"
#include "g2o/stuff/logger.h"

#ifdef G2O_HAVE_JSON
#include <nlohmann/json.hpp>

#include "io_wrapper_json.h"
#endif  // HAVE_JSON

namespace g2o {

#ifdef G2O_HAVE_JSON

std::optional<AbstractGraph> IoJson::load(std::istream& input) {
  try {
    nlohmann::json json;
    input >> json;
    return json::fromJson(json);
  } catch (const std::exception& e) {
    G2O_ERROR("Exception while saving: {}", e.what());
  }
  return std::nullopt;
}

bool IoJson::save(std::ostream& output, const AbstractGraph& graph) {
  try {
    output << std::setw(2);
    output << json::toJson(graph);
  } catch (const std::exception& e) {
    G2O_ERROR("Exception while saving: {}", e.what());
    return false;
  }
  return true;
}

#else

std::optional<AbstractGraph> IoJson::load(std::istream&) {
  G2O_WARN("Loading JSON is not supported");
  return std::nullopt;
}

bool IoJson::save(std::ostream&, const AbstractGraph&) {
  G2O_WARN("Saving JSON is not supported");
  return false;
}

#endif

}  // namespace g2o
