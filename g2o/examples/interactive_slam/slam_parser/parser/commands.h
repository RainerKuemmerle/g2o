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

#include <vector>
#include <string>

namespace SlamParser {

  enum CommandType
  {
    CT_ADD_NODE,
    CT_ADD_EDGE,
    CT_SOLVE_STATE,
    CT_QUERY_STATE,
    CT_FIX,
  };

  class CommandNode
  {
    public:
      CommandNode(CommandType commandType, const std::string& tag) : _commandType(commandType), _tag(tag) {}
      virtual ~CommandNode() {}
      CommandType commandType() const { return _commandType;}
      const std::string& tag() const { return _tag;}
    protected:
      CommandType _commandType;
      std::string _tag;
  };

  class AddNode : public CommandNode
  {
    public:
      AddNode(const std::string& tag, int id, int dimension, const std::vector<double>& values = std::vector<double>()) :
        CommandNode(CT_ADD_NODE, tag),
        _id(id), _dimension(dimension), _values(values)
    {
    }

      int id() const {return _id;}
      int dimension() const {return _dimension;}
      const std::vector<double>& values() { return _values;}

    protected:
      int _id;
      int _dimension;
      std::vector<double> _values;
  };

  class AddEdge : public CommandNode
  {
    public:
      AddEdge(const std::string& tag, int id, int dimension, int id1, int id2, const std::vector<double>& values, const std::vector<double> information) :
        CommandNode(CT_ADD_EDGE, tag),
        _id(id), _dimension(dimension), _id1(id1), _id2(id2), _values(values), _information(information)
    {
    }

      int id() const {return _id;}
      int dimension() const {return _dimension;}
      int id1() const {return _id1;}
      int id2() const {return _id2;}
      const std::vector<double>& values() { return _values;}
      const std::vector<double>& information() { return _information;}

    protected:
      int _id;
      int _dimension;
      int _id1;
      int _id2;
      std::vector<double> _values;
      std::vector<double> _information;
  };

  class SolveSate : public CommandNode
  {
    public:
      SolveSate(const std::string& tag) :
        CommandNode(CT_SOLVE_STATE, tag)
    {
    }
  };

  class QueryState : public CommandNode
  {
    public:
      explicit QueryState(const std::string& tag, const std::vector<int>& ids = std::vector<int>()) :
        CommandNode(CT_QUERY_STATE, tag), _ids(ids)
    {
    }
      const std::vector<int>& ids() { return _ids;}
    protected:
      std::vector<int> _ids;
  };

  class FixNode : public CommandNode
  {
    public:
      explicit FixNode(const std::string& tag, const std::vector<int>& ids) :
        CommandNode(CT_FIX, tag), _ids(ids)
    {
    }
      const std::vector<int>& ids() { return _ids;}
    protected:
      std::vector<int> _ids;
  };

} // end namespace

#endif
