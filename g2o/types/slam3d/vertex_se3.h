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

#ifndef G2O_VERTEX_SE3_
#define G2O_VERTEX_SE3_

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "isometry3d_mappings.h"
#include "g2o_types_slam3d_api.h"

namespace g2o {

/**
 * \brief 3D pose Vertex, represented as an Isometry3D
 *
 * 3D pose vertex, represented as an Isometry3D, i.e., an affine transformation
 * which is constructed by only concatenating rotation and translation
 * matrices. Hence, no scaling or projection.  To avoid that the rotational
 * part of the Isometry3D gets numerically unstable we compute the nearest
 * orthogonal matrix after a large number of calls to the oplus method.
 * 
 * The parameterization for the increments constructed is a 6d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 */
  class G2O_TYPES_SLAM3D_API VertexSE3 : public BaseVertex<6, Isometry3D>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      static const int orthogonalizeAfter = 1000; //< orthogonalize the rotation matrix after N updates

      VertexSE3();

      virtual void setToOriginImpl() {
        _estimate = Isometry3D::Identity();
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual bool setEstimateDataImpl(const double* est){
        Eigen::Map<const Vector7d> v(est);
        _estimate=internal::fromVectorQT(v);
        return true;
      }

      virtual bool getEstimateData(double* est) const{
        Eigen::Map<Vector7d> v(est);
        v=internal::toVectorQT(_estimate);
        return true;
      }

      virtual int estimateDimension() const {
        return 7;
      }

      virtual bool setMinimalEstimateDataImpl(const double* est){
        Eigen::Map<const Vector6d> v(est);
        _estimate = internal::fromVectorMQT(v);
        return true;
      }

      virtual bool getMinimalEstimateData(double* est) const{
        Eigen::Map<Vector6d> v(est);
        v = internal::toVectorMQT(_estimate);
        return true;
      }

      virtual int minimalEstimateDimension() const {
        return 6;
      }

      /**
       * update the position of this vertex. The update is in the form
       * (x,y,z,qx,qy,qz) whereas (x,y,z) represents the translational update
       * and (qx,qy,qz) corresponds to the respective elements. The missing
       * element qw of the quaternion is recovred by
       * || (qw,qx,qy,qz) || == 1 => qw = sqrt(1 - || (qx,qy,qz) ||
       */
      virtual void oplusImpl(const double* update)
      {
        Eigen::Map<const Vector6d> v(update);
        Isometry3D increment = internal::fromVectorMQT(v);
        _estimate = _estimate * increment;
        if (++_numOplusCalls > orthogonalizeAfter) {
          _numOplusCalls = 0;
          internal::approximateNearestOrthogonalMatrix(_estimate.matrix().topLeftCorner<3,3>());
        }
      }

      //! wrapper function to use the old SE3 type
      SE3Quat G2O_ATTRIBUTE_DEPRECATED(estimateAsSE3Quat() const) { return internal::toSE3Quat(estimate());}
      //! wrapper function to use the old SE3 type
      void G2O_ATTRIBUTE_DEPRECATED(setEstimateFromSE3Quat(const SE3Quat& se3)) { setEstimate(internal::fromSE3Quat(se3));}

    protected:
      int _numOplusCalls;     ///< store how often opluse was called to trigger orthogonaliation of the rotation matrix
  };

  /**
   * \brief write the vertex to some Gnuplot data file
   */
  class VertexSE3WriteGnuplotAction: public WriteGnuplotAction {
    public:
      VertexSE3WriteGnuplotAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
          HyperGraphElementAction::Parameters* params_ );
  };

#ifdef G2O_HAVE_OPENGL
  /**
   * \brief visualize the 3D pose vertex
   */
  class G2O_TYPES_SLAM3D_API VertexSE3DrawAction: public DrawAction{
    public:
      VertexSE3DrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_);
    protected:
      virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
      FloatProperty* _triangleX, *_triangleY;
  };
#endif

} // end namespace

#endif
