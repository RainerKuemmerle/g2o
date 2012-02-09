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

#include "parser_interface.h"

#include "slam_context_interface.h"
#include "slam_parser/parser/driver.h"

namespace SlamParser {

  ParserInterface::ParserInterface(AbstractSlamInterface* slamInterface)
  {
    _slamContextInterface = new SlamContextInterface(slamInterface);
    _driver = new Driver(*_slamContextInterface);
    //_driver->trace_parsing = true;
    //_driver->trace_scanning = true;
  }

  ParserInterface::~ParserInterface()
  {
    delete _slamContextInterface;
    delete _driver;
  }

  bool ParserInterface::parseCommand(std::istream& input)
  {
    if (input.eof())
      return false;
    _buffer.str("");
    _buffer.clear();
    input.get(*_buffer.rdbuf(), ';');
    if (! input.eof()) // get the ';'
      _buffer << (char)input.get();
    _driver->parse_stream(_buffer);
    return true;
  }

} // end namespace
