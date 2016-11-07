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

#include "types_slam3d_addons.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include <typeinfo>
#include <iostream>

namespace g2o {

  G2O_REGISTER_TYPE_GROUP(slam3d_addons);

  G2O_REGISTER_TYPE(VERTEX3, VertexSE3Euler);
  G2O_REGISTER_TYPE(EDGE3, EdgeSE3Euler);
  G2O_REGISTER_TYPE(VERTEX_PLANE, VertexPlane);
  G2O_REGISTER_TYPE(EDGE_SE3_PLANE_CALIB, EdgeSE3PlaneSensorCalib);

  G2O_REGISTER_TYPE(VERTEX_LINE3D, VertexLine3D);
  G2O_REGISTER_TYPE(EDGE_SE3_LINE3D, EdgeSE3Line3D);
  G2O_REGISTER_TYPE(EDGE_PLANE, EdgePlane);
  G2O_REGISTER_TYPE(EDGE_SE3_CALIB, EdgeSE3Calib);

#ifdef G2O_HAVE_OPENGL
  G2O_REGISTER_ACTION(CacheCameraDrawAction);
  G2O_REGISTER_ACTION(VertexPlaneDrawAction);  
  G2O_REGISTER_ACTION(EdgeSE3PlaneSensorCalibDrawAction);
  G2O_REGISTER_ACTION(VertexLine3DDrawAction);
  G2O_REGISTER_ACTION(EdgeSE3Line3DDrawAction);
#endif

  G2O_ATTRIBUTE_CONSTRUCTOR(init_slam3d_addons_types)
  {
    static bool initialized = false;
    if (initialized)
      return;
    initialized = true;

#ifdef G2O_HAVE_OPENGL
    HyperGraphActionLibrary* actionLib = HyperGraphActionLibrary::instance();
    HyperGraphElementAction* vertexse3eulerdraw=new g2o::VertexSE3DrawAction;
    vertexse3eulerdraw->setTypeName(typeid(VertexSE3Euler).name());
    actionLib->registerAction(vertexse3eulerdraw);

    HyperGraphElementAction* edgese3eulerdraw=new g2o::EdgeSE3DrawAction;
    edgese3eulerdraw->setTypeName(typeid(EdgeSE3Euler).name());
    actionLib->registerAction(edgese3eulerdraw);
#endif
  }

} // end namespace
