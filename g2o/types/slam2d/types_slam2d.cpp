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

#include "types_slam2d.h"

#include "g2o/core/factory.h"

#include "g2o/stuff/macros.h"

#include <iostream>

namespace g2o {

  namespace types_slam2d {
    int initialized = 0;

    void init()
    {
      if (types_slam2d::initialized)
        return;

      Factory* factory = Factory::instance();
      //std::cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << std::endl;

      factory->registerType("VERTEX_SE2", new HyperGraphElementCreator<VertexSE2>);
      factory->registerType("VERTEX_XY", new HyperGraphElementCreator<VertexPointXY>);
      factory->registerType("PARAMS_SE2OFFSET", new HyperGraphElementCreator<ParameterSE2Offset>);
      factory->registerType("CACHE_SE2_OFFSET", new HyperGraphElementCreator<CacheSE2Offset>);

      factory->registerType("EDGE_PRIOR_SE2", new HyperGraphElementCreator<EdgeSE2Prior>);
      factory->registerType("EDGE_SE2", new HyperGraphElementCreator<EdgeSE2>);
      factory->registerType("EDGE_SE2_XY", new HyperGraphElementCreator<EdgeSE2PointXY>);
      factory->registerType("EDGE_BEARING_SE2_XY", new HyperGraphElementCreator<EdgeSE2PointXYBearing>);
      factory->registerType("EDGE_SE2_XY_CALIB", new HyperGraphElementCreator<EdgeSE2PointXYCalib>);
      factory->registerType("EDGE_SE2_OFFSET", new HyperGraphElementCreator<EdgeSE2Offset>);
      factory->registerType("EDGE_SE2_POINTXY_OFFSET", new HyperGraphElementCreator<EdgeSE2PointXYOffset>);

      HyperGraphActionLibrary* actionLib = HyperGraphActionLibrary::instance();

      actionLib->registerAction(new VertexSE2WriteGnuplotAction);
      actionLib->registerAction(new VertexPointXYWriteGnuplotAction);
      actionLib->registerAction(new EdgeSE2WriteGnuplotAction);
      actionLib->registerAction(new EdgeSE2PointXYWriteGnuplotAction);
      actionLib->registerAction(new EdgeSE2PointXYBearingWriteGnuplotAction);

#ifdef G2O_HAVE_OPENGL
      actionLib->registerAction(new VertexSE2DrawAction);
      actionLib->registerAction(new VertexPointXYDrawAction);
      actionLib->registerAction(new EdgeSE2DrawAction);
      actionLib->registerAction(new EdgeSE2PointXYDrawAction);
      actionLib->registerAction(new EdgeSE2PointXYBearingDrawAction);
#endif

      types_slam2d::initialized = 1;

    }
  } // end namespace

  G2O_ATTRIBUTE_CONSTRUCTOR(init_types_slam2d)
  {
    types_slam2d::init();
  }

} // end namespace
