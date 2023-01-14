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

#ifndef G2O_GRAPH_PARAMETER_CONTAINER_HH_
#define G2O_GRAPH_PARAMETER_CONTAINER_HH_

#include <iosfwd>
#include <map>
#include <memory>
#include <string>

namespace g2o {

class Parameter;

/**
 * \brief map id to parameters
 */
class ParameterContainer : protected std::map<int, std::shared_ptr<Parameter>> {
 public:
  using BaseClass = std::map<int, std::shared_ptr<Parameter>>;

  /**
   * create a container for the parameters.
   */
  ParameterContainer() = default;
  virtual ~ParameterContainer() = default;
  //! add parameter to the container
  bool addParameter(std::shared_ptr<Parameter> p);
  //! return a parameter based on its ID
  std::shared_ptr<Parameter> getParameter(int id) const;
  //! remove a parameter from the container and returns the formerly stored
  //! parameter
  std::shared_ptr<Parameter> detachParameter(int id);
  //! read parameters from a stream
  virtual bool read(
      std::istream& is,
      const std::map<std::string, std::string>* renamedTypesLookup = nullptr);
  //! write the data to a stream
  virtual bool write(std::ostream& os) const;

  // stuff of the base class that should re-appear
  using BaseClass::clear;
  using BaseClass::size;
};

}  // namespace g2o

#endif
