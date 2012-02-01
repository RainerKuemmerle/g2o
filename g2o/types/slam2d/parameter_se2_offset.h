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

#ifndef G2O_VERTEX_SE2_OFFSET_PARAMETERS_H_
#define G2O_VERTEX_SE2_OFFSET_PARAMETERS_H_

#include "g2o/core/optimizable_graph.h"

#include "se2.h"
#include "g2o_types_slam2d_api.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/cache.h"

#include <Eigen/Geometry>

namespace g2o {

  class VertexSE2;

  /**
   * \brief offset for an SE2
   */
  class G2O_TYPES_SLAM2D_API ParameterSE2Offset: public Parameter
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      ParameterSE2Offset();

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      /**
       * update the offset to a new value.
       * re-calculates the different representations, e.g., the rotation matrix
       */
      void setOffset(const SE2& offset_ = SE2());

      const SE2& offset() const { return _offset;}

      //! rotation of the offset as 2x2 rotation matrix
      const Eigen::Isometry2d& offsetMatrix() const { return _offsetMatrix;}

      //! rotation of the inverse offset as 2x2 rotation matrix
      const Eigen::Isometry2d& inverseOffsetMatrix() const { return _inverseOffsetMatrix;}

    protected:
      SE2 _offset;
      Eigen::Isometry2d _offsetMatrix;
      Eigen::Isometry2d _inverseOffsetMatrix;
  };

  /**
   * \brief caching the offset related to a vertex
   */
  class G2O_TYPES_SLAM2D_API CacheSE2Offset: public Cache {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      CacheSE2Offset();
      virtual void updateImpl();

      const ParameterSE2Offset* offsetParam() const { return _offsetParam;}
      void setOffsetParam(ParameterSE2Offset* offsetParam);

      const SE2& w2n() const {return _se2_w2n;}
      const SE2& n2w() const {return _se2_n2w;}

      const Eigen::Isometry2d& w2nMatrix() const { return _w2n;}
      const Eigen::Isometry2d& n2wMatrix() const { return _n2w;}
      const Eigen::Isometry2d& w2lMatrix() const { return _w2l;}

      const Eigen::Matrix2d RpInverseRInverseMatrix() const { return _RpInverse_RInverse; }
      const Eigen::Matrix2d RpInverseRInversePrimeMatrix() const { return _RpInverse_RInversePrime; }

    protected:
      ParameterSE2Offset* _offsetParam; ///< the parameter connected to the cache
      SE2 _se2_w2n;
      SE2 _se2_n2w;

      Eigen::Isometry2d _w2n; ///< world to sensor transform
      Eigen::Isometry2d _w2l; ///< world to local
      Eigen::Isometry2d _n2w; ///< sensor to world
      Matrix2d _RpInverse_RInverse;
      Matrix2d _RpInverse_RInversePrime;
      
    protected:
      virtual bool resolveDependancies();
      
  };

}

#endif
