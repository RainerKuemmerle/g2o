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

#ifndef G2O_CAMERA_PARAMETERS_H_
#define G2O_CAMERA_PARAMETERS_H_

#include "g2o/core/hyper_graph_action.h"
#include "parameter_se3_offset.h"
#include "g2o_types_slam3d_api.h"

namespace g2o {
  /**
   * \brief parameters for a camera
   */
  class G2O_TYPES_SLAM3D_API ParameterCamera: public ParameterSE3Offset {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      ParameterCamera();
      void setKcam(double fx, double fy, double cx, double cy);
      void setOffset(const Eigen::Isometry3d& offset_ = Eigen::Isometry3d::Identity());

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      const Eigen::Matrix3d& Kcam() const { return _Kcam;}
      const Eigen::Matrix3d& invKcam() const { return _invKcam;}
      const Eigen::Matrix3d& Kcam_inverseOffsetR() const { return _Kcam_inverseOffsetR;}

    protected:
      Eigen::Matrix3d _Kcam;
      Eigen::Matrix3d _invKcam;
      Eigen::Matrix3d _Kcam_inverseOffsetR;
  };

  class G2O_TYPES_SLAM3D_API CacheCamera: public CacheSE3Offset {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    //! parameters of the camera
    const ParameterCamera* camParams() const {return params;}
    //! return the world to image transform
    const Eigen::Affine3d& w2i() const {return _w2i;}
  protected:
    virtual void updateImpl();
    virtual bool resolveDependancies();
    Eigen::Affine3d _w2i; ///< world to image transform
    ParameterCamera* params;
  };


#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM3D_API CacheCameraDrawAction: public DrawAction{
    public:
      CacheCameraDrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_ );
    protected:
      virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
      FloatProperty* _cameraZ, *_cameraSide;
  };
#endif

} // end namespace

#endif
