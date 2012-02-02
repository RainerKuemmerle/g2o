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

#include "types_slam2d_online.h"
#include "types_slam3d_online.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include <iostream>

namespace g2o {
  using namespace std;

  G2O_ATTRIBUTE_CONSTRUCTOR(init_types_interactive_online)
  {
    Factory* factory = Factory::instance();
    //cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << endl;

    factory->registerType("ONLINE_EDGE_SE2", new HyperGraphElementCreator<OnlineEdgeSE2>);
    factory->registerType("ONLINE_VERTEX_SE2", new HyperGraphElementCreator<OnlineVertexSE2>);

    factory->registerType("ONLINE_VERTEX_SE3:QUAT", new HyperGraphElementCreator<OnlineVertexSE3>);
    factory->registerType("ONLINE_EDGE_SE3:QUAT", new HyperGraphElementCreator<OnlineEdgeSE3>);
  }

} // end namespace
