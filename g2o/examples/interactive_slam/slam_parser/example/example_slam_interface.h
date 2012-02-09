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

#ifndef EXAMPLE_SLAM_INTERFACE_H
#define EXAMPLE_SLAM_INTERFACE_H

#include "slam_parser/interface/abstract_slam_interface.h"

#include <map>
#include <vector>

/**
 * \brief example for an interface to a SLAM algorithm
 *
 * Example for an interface to a SLAM algorithm. You may modify this class to fit your needs.
 * The example class does not actually perform any optimization, it just keeps the input values
 * and outputs the same values if asked. See the documentation of SlamParser::AbstractSlamInterface
 * for details.
 */
class ExampleSlamInterface : public SlamParser::AbstractSlamInterface
{
  public:
    ExampleSlamInterface();

    bool addNode(const std::string& tag, int id, int dimension, const std::vector<double>& values);

    bool addEdge(const std::string& tag, int id, int dimension, int v1, int v2, const std::vector<double>& measurement, const std::vector<double>& information);

    bool fixNode(const std::vector<int>& nodes);

    bool queryState(const std::vector<int>& nodes);

    bool solveState();

  protected:
    /* add variables to control the SLAM algorithm or for other general requirements */
    std::map<int, std::pair<std::string, std::vector<double> > > _vertices; ///< the original value of the input (actually not needed if a real SLAM engine is running)
};

#endif
