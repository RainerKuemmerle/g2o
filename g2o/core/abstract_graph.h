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

#ifndef G2O_CORE_ABSTRACT_GRAPH_H
#define G2O_CORE_ABSTRACT_GRAPH_H

#include <istream>
#include <string>
#include <utility>
#include <vector>

namespace g2o {

class OptimizableGraph;

/**
 * @brief An abstract graph holding the data of vertices and edges
 *
 * An abstract graph which holds all the data of a optimization problem
 * represented as a factor graph. It consists of nodes and edges. Additionally,
 * each node/edge can hold a list of data. Furthermore, parameters can be
 * stored. Whereas a parameter represents an agnostic part of the optimization
 * problem such as parameters of a camera.
 */
class AbstractGraph {
 public:
  struct AbstractData {
    std::string tag;   ///< the tag of this data
    std::string data;  ///< the serialized data as a string
    AbstractData() = default;
    AbstractData(std::string tag, std::string data)
        : tag(std::move(tag)), data(std::move(data)) {}
  };

  struct AbstractParameter {
    std::string tag;            ///< the tag of this parameter
    int id;                     ///< its ID
    std::vector<double> value;  ///< its value as a vector
    AbstractParameter() = default;
    AbstractParameter(std::string tag, int id, std::vector<double> value)
        : tag(std::move(tag)), id(id), value(std::move(value)) {}
  };

  struct AbstractGraphElement {
    std::string tag;
    std::vector<AbstractData> data;  ///< the associated data
    AbstractGraphElement() = default;
    AbstractGraphElement(std::string tag, std::vector<AbstractData> data)
        : tag(std::move(tag)), data(std::move(data)) {}
  };

  struct AbstractVertex : public AbstractGraphElement {
    int id;                        ///< its ID
    std::vector<double> estimate;  ///< the estimate as a vector
    AbstractVertex() = default;
    AbstractVertex(std::string tag, int id, std::vector<double> estimate,
                   std::vector<AbstractData> data = {})
        : AbstractGraphElement(std::move(tag), std::move(data)),
          id(id),
          estimate(std::move(estimate)) {}
  };

  struct AbstractEdge : public AbstractGraphElement {
    std::vector<int> ids;  ///< the ids of the vertices connected by this edge
    std::vector<int> param_ids;  ///< the ids of the parameters of this edge
    std::vector<double> measurement;  ///< the measurement as a vector
    std::vector<double>
        information;  ///< upper triangular part of the information matrix
    AbstractEdge() = default;
    AbstractEdge(std::string tag, std::vector<int> ids,
                 std::vector<double> measurement,
                 std::vector<double> information,
                 std::vector<int> param_ids = {},
                 std::vector<AbstractData> data = {})
        : AbstractGraphElement(std::move(tag), std::move(data)),
          ids(std::move(ids)),
          param_ids(std::move(param_ids)),
          measurement(std::move(measurement)),
          information(std::move(information)) {}
  };

  //! Possible formats for loading and saving
  enum class Format { kG2O = 0, kBinary = 1, kJson = 2, kXML = 3 };

  AbstractGraph() = default;
  ~AbstractGraph() = default;

  bool load(const std::string& filename,
            Format format = AbstractGraph::Format::kG2O);
  bool load(std::istream& input, Format format = AbstractGraph::Format::kG2O);

  [[nodiscard]] bool save(const std::string& filename,
                          Format format = AbstractGraph::Format::kG2O) const;
  [[nodiscard]] bool save(std::ostream& output,
                          Format format = AbstractGraph::Format::kG2O) const;

  std::vector<int>& fixed() { return fixed_; };
  [[nodiscard]] const std::vector<int>& fixed() const { return fixed_; };

  std::vector<AbstractParameter>& parameters() { return parameters_; };
  [[nodiscard]] const std::vector<AbstractParameter>& parameters() const {
    return parameters_;
  };

  std::vector<AbstractVertex>& vertices() { return vertices_; };
  [[nodiscard]] const std::vector<AbstractVertex>& vertices() const {
    return vertices_;
  };

  std::vector<AbstractEdge>& edges() { return edges_; };
  [[nodiscard]] const std::vector<AbstractEdge>& edges() const {
    return edges_;
  };

  void clear();

 protected:
  std::vector<int> fixed_;
  std::vector<AbstractParameter> parameters_;
  std::vector<AbstractVertex> vertices_;
  std::vector<AbstractEdge> edges_;
};
}  // namespace g2o

#endif
