// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
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

#ifndef G2O_SEVEN_DOF_EXPMAP_TYPES
#define G2O_SEVEN_DOF_EXPMAP_TYPES

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "sim3.h"
#include "types_seven_dof_expmap_api.h"

namespace g2o {

#ifdef _MSC_VER
// explicit instantiation of BaseVertex, if not instantiated causes already
// defined error in some cases (msvc debug only) see links below
// https://stackoverflow.com/questions/44960760/msvc-dll-exporting-class-that-inherits-from-template-cause-lnk2005-already-defin
// https://developercommunity.visualstudio.com/content/problem/224597/linker-failing-because-of-multiple-definitions-of.html
template class BaseVertex<7, Sim3>;
#endif

/**
 * \brief Sim3 Vertex, (x,y,z,qw,qx,qy,qz)
 * the parameterization for the increments constructed is a 7d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 *
 * Will represent relative transformation between two cameras
 */
class G2O_TYPES_SIM3_API VertexSim3Expmap : public BaseVertex<7, Sim3> {
 public:
  VertexSim3Expmap();

  void oplusImpl(const VectorX::MapType& update) override;

  Vector2 _principle_point1, _principle_point2;
  Vector2 _focal_length1, _focal_length2;

  [[nodiscard]] Vector2 cam_map1(const Vector2& v) const;

  [[nodiscard]] Vector2 cam_map2(const Vector2& v) const;

  bool _fix_scale;

 protected:
};

/**
 * \brief 7D edge between two Vertex7
 */
class G2O_TYPES_SIM3_API EdgeSim3
    : public BaseBinaryEdge<7, Sim3, VertexSim3Expmap, VertexSim3Expmap> {
 public:
  void computeError() override;

  double initialEstimatePossible(const OptimizableGraph::VertexSet&,
                                 OptimizableGraph::Vertex*) override;
  void initialEstimate(const OptimizableGraph::VertexSet& from,
                       OptimizableGraph::Vertex* /*to*/) override;
#if G2O_SIM3_JACOBIAN
  virtual void linearizeOplus();
#endif
};

/**/
class G2O_TYPES_SIM3_API EdgeSim3ProjectXYZ
    : public BaseBinaryEdge<2, Vector2, VertexPointXYZ, VertexSim3Expmap> {
 public:
  void computeError() override;

  // virtual void linearizeOplus();
};

/**/
class G2O_TYPES_SIM3_API EdgeInverseSim3ProjectXYZ
    : public BaseBinaryEdge<2, Vector2, VertexPointXYZ, VertexSim3Expmap> {
 public:
  void computeError() override;

  // virtual void linearizeOplus();
};

}  // namespace g2o

#endif
