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

#ifndef G2O_VERTEX_SE3_OFFSET_PARAMETERS_H_
#define G2O_VERTEX_SE3_OFFSET_PARAMETERS_H_

#include "g2o/core/optimizable_graph.h"

#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/cache.h"
#include "g2o_types_slam3d_api.h"

#include <Eigen/Geometry>

namespace g2o {

  class VertexSE3;

  /**
   * \brief offset for an SE3
   */
  class G2O_TYPES_SLAM3D_API ParameterSE3Offset: public Parameter
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      ParameterSE3Offset();

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      /**
       * update the offset to a new value.
       * re-calculates the different representations, e.g., the rotation matrix
       */
      void setOffset(const Isometry3D& offset_=Isometry3D::Identity());

      //! rotation of the offset as 3x3 rotation matrix
      const Isometry3D& offset() const { return _offset;}

      //! rotation of the inverse offset as 3x3 rotation matrix
      const Isometry3D& inverseOffset() const { return _inverseOffset;}

    protected:
      Isometry3D _offset;
      Isometry3D _inverseOffset;
  };

  /**
   * \brief caching the offset related to a vertex
   */
  class G2O_TYPES_SLAM3D_API CacheSE3Offset: public Cache {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      CacheSE3Offset();
      virtual void updateImpl();

      const ParameterSE3Offset* offsetParam() const { return _offsetParam;}
      void setOffsetParam(ParameterSE3Offset* offsetParam);

      const Isometry3D& w2n() const { return _w2n;}
      const Isometry3D& n2w() const { return _n2w;}
      const Isometry3D& w2l() const { return _w2l;}

    protected:
      ParameterSE3Offset* _offsetParam; ///< the parameter connected to the cache
      Isometry3D _w2n;
      Isometry3D _n2w;
      Isometry3D _w2l;

    protected:
      virtual bool resolveDependancies();
  };


#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM3D_API CacheSE3OffsetDrawAction: public DrawAction{
    public:
      CacheSE3OffsetDrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
          HyperGraphElementAction::Parameters* params_ );
    protected:
      virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
      FloatProperty* _cubeSide;
  };
#endif

}

#endif
