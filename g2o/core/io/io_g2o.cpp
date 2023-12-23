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

#include "io_g2o.h"

#include <iostream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "g2o/core/abstract_graph.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/logger.h"
#include "g2o/stuff/logger_format.h"  // IWYU pragma: keep
#include "g2o/stuff/string_tools.h"

namespace {
std::ostream& operator<<(std::ostream& os, const std::vector<double>& v) {
  for (size_t i = 0; i < v.size(); ++i) {
    if (i > 0) os << " ";
    os << v[i];
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<int>& v) {
  for (size_t i = 0; i < v.size(); ++i) {
    if (i > 0) os << " ";
    os << v[i];
  }
  return os;
}
}  // namespace

namespace g2o {

AbstractGraph IoG2O::load(std::istream& input) {
  Factory* factory = Factory::instance();
  std::unordered_set<std::string> warned_unknown_types;

  AbstractGraph result;
  AbstractGraph::AbstractGraphElement* last_data_container = nullptr;

  std::stringstream current_line;
  std::string token;

  int line_number = 0;
  while (true) {
    const int bytesRead = readLine(input, current_line);
    line_number++;
    if (bytesRead == -1) break;
    current_line >> token;
    if (bytesRead == 0 || token.empty() || token[0] == '#') continue;

    // handle FIX command encoded in the input
    if (token == "FIX") {
      int id;
      while (current_line >> id) {
        result.fixed().emplace_back(id);
        G2O_TRACE("Fixing vertex {}", id);
      }
      continue;
    }

    token = mapType(token);
    G2O_TRACE("Reading token {}", token);
    Factory::TypeInfo type_info = factory->typeInfo(token);
    if (type_info.elementTypeBit < 0) {
      if (warned_unknown_types.count(token) != 1) {
        warned_unknown_types.insert(token);
        G2O_ERROR("Unknown type {} in line {}", token, line_number);
      }
      continue;
    }

    // first handle the parameters
    if (type_info.elementTypeBit == HyperGraph::kHgetParameter) {
      AbstractGraph::AbstractParameter parameter;
      parameter.tag = token;
      current_line >> parameter.id;
      double value;
      while (current_line >> value) parameter.value.emplace_back(value);
      result.parameters().emplace_back(parameter);
      continue;
    }

    // it's a vertex type
    if (type_info.elementTypeBit == HyperGraph::kHgetVertex) {
      AbstractGraph::AbstractVertex vertex;
      vertex.tag = token;
      current_line >> vertex.id;
      // read estimate vector
      vertex.estimate.resize(type_info.dimension);
      for (auto& v : vertex.estimate) current_line >> v;
      if (!current_line) {
        G2O_ERROR("Error reading vertex {} at line {}", token, line_number);
        continue;
      }
      result.vertices().emplace_back(vertex);
      G2O_TRACE("Read vertex {} with estimate [{}]", vertex.tag,
                fmt::join(vertex.estimate, ","));
      last_data_container = &result.vertices().back();
    } else if (type_info.elementTypeBit == HyperGraph::kHgetEdge) {
      AbstractGraph::AbstractEdge edge;
      edge.tag = token;
      if (type_info.number_vertices > 0) {
        edge.ids.resize(type_info.number_vertices);
        for (auto& id : edge.ids) current_line >> id;
      } else {  // reading the IDs of a dynamically sized edge
        std::string buff;
        while (current_line >> buff) {
          if (buff == "||") break;
          edge.ids.push_back(stoi(buff));
        }
      }
      // read the parameter ids
      if (type_info.number_parameters > 0) {
        edge.param_ids.resize(type_info.number_parameters);
        for (auto& param_id : edge.param_ids) current_line >> param_id;
      }
      // read measurement vector
      edge.measurement.resize(type_info.dimension);
      for (auto& v : edge.measurement) current_line >> v;
      // read upper triangle of the information matrix
      const int& min_dim = type_info.minimal_dimension;
      edge.information.resize((min_dim * (min_dim + 1)) / 2);
      for (auto& v : edge.information) current_line >> v;

      if (!current_line) {
        G2O_ERROR("Error reading edge {} at line {}", token, line_number);
        continue;
      }
      result.edges().emplace_back(edge);
      G2O_TRACE(
          "Read edge {} connecting [{}] with measurement [{}] and information "
          "[{}]",
          edge.tag, fmt::join(edge.ids, ","), fmt::join(edge.measurement, ","),
          fmt::join(edge.information, ","));
      last_data_container = &result.edges().back();
    } else if (type_info.elementTypeBit == HyperGraph::kHgetData) {
      if (!last_data_container) {
        G2O_ERROR(
            "Error reading data {} at line {}: got data element, but no data "
            "container available",
            token, line_number);
      }
      AbstractGraph::AbstractData data;
      data.tag = token;
      std::stringstream remaining_line;
      remaining_line << current_line.rdbuf();
      data.data = trim(remaining_line.str());
      G2O_TRACE("Read data {} with content {}", data.tag, data.data);
      last_data_container->data.emplace_back(data);
    }
  }

  return result;
}

bool IoG2O::save(std::ostream& output, const AbstractGraph& graph) {
  if (!graph.fixed().empty()) {
    output << "FIX " << graph.fixed() << '\n';
  }

  for (const auto& param : graph.parameters()) {
    output << param.tag << " " << param.id << " " << param.value << '\n';
  }

  auto printData = [](std::ostream& output,
                      const std::vector<AbstractGraph::AbstractData>& data) {
    for (const auto& d : data) {
      output << d.tag << " " << d.data << '\n';
    }
  };

  for (const auto& vertex : graph.vertices()) {
    output << vertex.tag << " " << vertex.id << " " << vertex.estimate << '\n';
    printData(output, vertex.data);
  }

  for (const auto& edge : graph.edges()) {
    output << edge.tag << " ";
    if (!edge.param_ids.empty()) output << edge.param_ids << " ";
    output << edge.ids << " " << edge.measurement << " " << edge.information
           << "\n";
    printData(output, edge.data);
  }

  return output.good();
}

}  // namespace g2o
