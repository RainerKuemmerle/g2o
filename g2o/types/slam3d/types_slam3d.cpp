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

#include "types_slam3d.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include <iostream>

namespace g2o {
  using namespace std;

  namespace types_slam3d {
    int initialized = 0;

    void init()
    {
      if (types_slam3d::initialized)
        return;
      Factory* factory = Factory::instance();
      //std::cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << std::endl;
      factory->registerType("VERTEX_SE3:QUAT", new HyperGraphElementCreator<VertexSE3>);
      factory->registerType("EDGE_SE3:QUAT", new HyperGraphElementCreator<EdgeSE3>);
      factory->registerType("VERTEX_TRACKXYZ", new HyperGraphElementCreator<VertexPointXYZ>);

      factory->registerType("PARAMS_SE3OFFSET", new HyperGraphElementCreator<ParameterSE3Offset>);
      factory->registerType("EDGE_SE3_TRACKXYZ", new HyperGraphElementCreator<EdgeSE3PointXYZ>);
      factory->registerType("EDGE_SE3_PRIOR", new HyperGraphElementCreator<EdgeSE3Prior>);
      factory->registerType("CACHE_SE3_OFFSET", new HyperGraphElementCreator<CacheSE3Offset>);
      factory->registerType("EDGE_SE3_OFFSET", new HyperGraphElementCreator<EdgeSE3Offset>);

      factory->registerType("PARAMS_CAMERACALIB", new HyperGraphElementCreator<ParameterCamera>);
      factory->registerType("CACHE_CAMERA", new HyperGraphElementCreator<CacheCamera>);
      factory->registerType("EDGE_PROJECT_DISPARITY", new HyperGraphElementCreator<EdgeSE3PointXYZDisparity>);
      factory->registerType("EDGE_PROJECT_DEPTH", new HyperGraphElementCreator<EdgeSE3PointXYZDepth>);


      HyperGraphActionLibrary* actionLib = HyperGraphActionLibrary::instance();
      actionLib->registerAction(new VertexSE3WriteGnuplotAction);
      actionLib->registerAction(new EdgeSE3WriteGnuplotAction);

#ifdef G2O_HAVE_OPENGL
      actionLib->registerAction(new VertexPointXYZDrawAction);
      actionLib->registerAction(new VertexSE3DrawAction);
      actionLib->registerAction(new EdgeSE3DrawAction);
      actionLib->registerAction(new CacheCameraDrawAction);
#endif
      types_slam3d::initialized = 1;
    }
  }

  G2O_ATTRIBUTE_CONSTRUCTOR(init_slam3d_types)
  {
    types_slam3d::init();
  }

} // end namespace
