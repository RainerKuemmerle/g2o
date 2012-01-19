#ifndef G2O_CAMERA_PARAMETERS_H_
#define G2O_CAMERA_PARAMETERS_H_

#include "se3quat.h"
#include "g2o/core/hyper_graph_action.h"
#include "parameter_se3_offset.h"

namespace g2o {

  /**
   * \brief parameters for a camera
   */
  class G2O_TYPES_SLAM3D_API ParameterCamera: public ParameterSE3Offset {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      ParameterCamera();
      void setKcam(double fx, double fy, double cx, double cy);
      void setOffset(const SE3Quat& offset_ = SE3Quat());

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
