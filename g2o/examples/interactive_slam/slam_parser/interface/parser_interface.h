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

#ifndef PARSER_INTERFACE_H
#define PARSER_INTERFACE_H

#include <iosfwd>
#include <sstream>

#include "abstract_slam_interface.h"

namespace SlamParser {

class SlamContextInterface;
class Driver;

/**
 * \brief top-level interface to the parser
 */
class ParserInterface {
 public:
  /**
   * construct a parser and use the given AbstractSlamInterface to communicate with the SLAM algorithm.
   */
  explicit ParserInterface(AbstractSlamInterface* slamInterface);
  ParserInterface& operator=(const ParserInterface&) = delete;
  ParserInterface(const ParserInterface&) = delete;
  virtual ~ParserInterface();

  /**
   * parse a single command and forward to the SLAM engine
   */
  bool parseCommand(std::istream& input);

 protected:
  SlamContextInterface* _slamContextInterface;
  Driver* _driver;
  std::stringstream _buffer;
};

}  // namespace SlamParser

#endif
