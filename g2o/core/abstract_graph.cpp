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

#include "abstract_graph.h"

#include <fstream>
#include <memory>
#include <optional>

#include "g2o/core/io/io_format.h"
#include "g2o/core/io/io_json.h"
#include "g2o/stuff/logger.h"
#include "io/io_g2o.h"

namespace {
std::string_view to_string(g2o::io::Format format) {
  switch (format) {
    case g2o::io::Format::kG2O:
      return "G2O";
    case g2o::io::Format::kBinary:
      return "Binary";
    case g2o::io::Format::kJson:
      return "JSON";
    case g2o::io::Format::kXML:
      return "XML";
  }
}

std::unique_ptr<g2o::IoInterface> allocate(g2o::io::Format format) {
  switch (format) {
    case g2o::io::Format::kG2O:
      return std::make_unique<g2o::IoG2O>();
    case g2o::io::Format::kBinary:
      break;
    case g2o::io::Format::kJson:
      return std::make_unique<g2o::IoJson>();
    case g2o::io::Format::kXML:
      break;
  }
  G2O_CRITICAL("Failed to create graph IO interface for format {}",
               to_string(format));
  return nullptr;
}

}  // namespace

namespace g2o {

bool AbstractGraph::load(const std::string& filename, io::Format format) {
  std::ifstream file_input(filename);
  return load(file_input, format);
}

bool AbstractGraph::load(std::istream& input, io::Format format) {
  std::unique_ptr<g2o::IoInterface> loader_interface = allocate(format);
  if (!loader_interface) return false;
  std::optional<AbstractGraph> load_result = loader_interface->load(input);
  if (!load_result.has_value()) {
    G2O_ERROR("Failure while loading graph, result will be empty");
    return false;
  }
  *this = std::move(load_result.value());
  return true;
}

bool AbstractGraph::save(const std::string& filename, io::Format format) const {
  std::ofstream file_output(filename);
  return save(file_output, format);
}

bool AbstractGraph::save(std::ostream& output, io::Format format) const {
  std::unique_ptr<g2o::IoInterface> loader_interface = allocate(format);
  if (!loader_interface) return false;
  return loader_interface->save(output, *this);
}

void AbstractGraph::clear() {
  fixed_.clear();
  parameters_.clear();
  vertices_.clear();
  edges_.clear();
}

}  // namespace g2o
