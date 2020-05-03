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

#ifndef ABSTRACT_SLAM_INTERFACE_H
#define ABSTRACT_SLAM_INTERFACE_H

#include <string>
#include <vector>

namespace SlamParser {

/**
 * \brief interface for communicating with the SLAM algorithm
 *
 * This interface allows the parser to communicate with the SLAM algorithm.
 */
class AbstractSlamInterface {
 public:
  /**
   * adding a node to the SLAM engine.
   * @param tag: the tag specifying the type of the vertex
   * @param id: the unique id of the node.
   * @param dimension: the dimension of the node.
   * @param values: the pose of the node, may be empty (i.e., the engine should initialize the node itself)
   * @return true, if adding was successful
   */
  virtual bool addNode(const std::string& tag, int id, int dimension, const std::vector<double>& values) = 0;

  /**
   * adding an edge to the SLAM engine.
   * @param tag: the tag specifying the type of the vertex
   * @param id: the unique id of the edge.
   * @param dimension: the dimension of the edge.
   * @param v1: the unique id of the edge of the first vertex
   * @param v2: the unique id of the edge of the second vertex
   * @param measurement: the measurement of the constraint
   * @param information: the information matrix (inverse of the covariance) representing the uncertainty of the
   * measurement (row-major upper triangular and diagonal)
   * @return true, if adding was successful
   */
  virtual bool addEdge(const std::string& tag, int id, int dimension, int v1, int v2,
                       const std::vector<double>& measurement, const std::vector<double>& information) = 0;

  /**
   * set some nodes to a fixed position
   * @param nodes: the list of vertex IDs to fix
   * @return true, if successful
   */
  virtual bool fixNode(const std::vector<int>& nodes) = 0;

  /**
   * Ask the SLAM engine to print the current estimate of the variables
   * @param nodes: the list of vertex IDs to print, if empty print all variables
   * @return true, if successful
   */
  virtual bool queryState(const std::vector<int>& nodes) = 0;

  /**
   * ask the engine to solve
   * @return true, if successful
   */
  virtual bool solveState() = 0;
};

}  // namespace SlamParser

#endif
