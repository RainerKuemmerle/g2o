#ifndef G2O_VERTEX_SE3_OFFSET_PARAMETERS_H_
#define G2O_VERTEX_SE3_OFFSET_PARAMETERS_H_

#include "g2o/core/optimizable_graph.h"

#include "se3quat.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/cache.h"

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
      void setOffset(const SE3Quat& offset_ = SE3Quat());

      //! return the offset as SE3Quat
      const SE3Quat& offset() const { return _offset;}

      //! rotation of the offset as 3x3 rotation matrix
      const Eigen::Isometry3d& offsetMatrix() const { return _offsetMatrix;}

      //! rotation of the inverse offset as 3x3 rotation matrix
      const Eigen::Isometry3d& inverseOffsetMatrix() const { return _inverseOffsetMatrix;}

    protected:
      SE3Quat _offset;
      Eigen::Isometry3d _offsetMatrix;
      Eigen::Isometry3d _inverseOffsetMatrix;
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

      const SE3Quat& w2n() const {return _se3_w2n;}
      const SE3Quat& n2w() const {return _se3_n2w;}

      const Eigen::Isometry3d& w2nMatrix() const { return _w2n;}
      const Eigen::Isometry3d& n2wMatrix() const { return _n2w;}
      const Eigen::Isometry3d& w2lMatrix() const { return _w2l;}

    protected:
      ParameterSE3Offset* _offsetParam; ///< the parameter connected to the cache
      SE3Quat _se3_w2n;
      SE3Quat _se3_n2w;

      Eigen::Isometry3d _w2n; ///< world to sensor transform
      Eigen::Isometry3d _w2l; ///< world to local
      Eigen::Isometry3d _n2w; ///< sensor to world

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
