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

#ifndef COMMANDS_H
#define COMMANDS_H

#include <string>
#include <utility>
#include <vector>

namespace slam_parser {

enum CommandType {
  kCtAddNode,
  kCtAddEdge,
  kCtSolveState,
  kCtQueryState,
  kCtFix,
};

class CommandNode {
 public:
  CommandNode(CommandType commandType, std::string tag)
      : commandType_(commandType), tag_(std::move(std::move(tag))) {}
  virtual ~CommandNode() = default;
  CommandType commandType() const { return commandType_; }
  const std::string& tag() const { return tag_; }

 protected:
  CommandType commandType_;
  std::string tag_;
};

class AddNode : public CommandNode {
 public:
  AddNode(const std::string& tag, int id, int dimension,
          std::vector<double> values = std::vector<double>())
      : CommandNode(kCtAddNode, tag),
        id_(id),
        dimension_(dimension),
        values_(std::move(std::move(values))) {}

  int id() const { return id_; }
  int dimension() const { return dimension_; }
  const std::vector<double>& values() { return values_; }

 protected:
  int id_;
  int dimension_;
  std::vector<double> values_;
};

class AddEdge : public CommandNode {
 public:
  AddEdge(const std::string& tag, int id, int dimension, int id1, int id2,
          std::vector<double> values, std::vector<double> information)
      : CommandNode(kCtAddEdge, tag),
        id_(id),
        dimension_(dimension),
        id1_(id1),
        id2_(id2),
        values_(std::move(std::move(values))),
        information_(std::move(std::move(information))) {}

  int id() const { return id_; }
  int dimension() const { return dimension_; }
  int id1() const { return id1_; }
  int id2() const { return id2_; }
  const std::vector<double>& values() { return values_; }
  const std::vector<double>& information() { return information_; }

 protected:
  int id_;
  int dimension_;
  int id1_;
  int id2_;
  std::vector<double> values_;
  std::vector<double> information_;
};

class SolveSate : public CommandNode {
 public:
  explicit SolveSate(const std::string& tag)
      : CommandNode(kCtSolveState, tag) {}
};

class QueryState : public CommandNode {
 public:
  explicit QueryState(const std::string& tag,
                      std::vector<int> ids = std::vector<int>())
      : CommandNode(kCtQueryState, tag), ids_(std::move(std::move(ids))) {}
  const std::vector<int>& ids() { return ids_; }

 protected:
  std::vector<int> ids_;
};

class FixNode : public CommandNode {
 public:
  explicit FixNode(const std::string& tag, std::vector<int> ids)
      : CommandNode(kCtFix, tag), ids_(std::move(std::move(ids))) {}
  const std::vector<int>& ids() { return ids_; }

 protected:
  std::vector<int> ids_;
};

}  // namespace slam_parser

#endif
