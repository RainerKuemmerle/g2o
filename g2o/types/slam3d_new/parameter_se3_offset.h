#ifndef G2O_VERTEX_SE3_OFFSET_PARAMETERS_NEW_H_
#define G2O_VERTEX_SE3_OFFSET_PARAMETERS_NEW_H_

#include "g2o/core/optimizable_graph.h"

#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/cache.h"

#include <Eigen/Geometry>

namespace Slam3dNew {
  using namespace g2o;

  class VertexSE3;

  /**
   * \brief offset for an SE3
   */
  class ParameterSE3Offset: public Parameter
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
      void setOffset(const Eigen::Isometry3d& offset_=Eigen::Isometry3d::Identity());

      //! rotation of the offset as 3x3 rotation matrix
      const Eigen::Isometry3d& offset() const { return _offset;}

      //! rotation of the inverse offset as 3x3 rotation matrix
      const Eigen::Isometry3d& inverseOffset() const { return _inverseOffset;}

    protected:
      Eigen::Isometry3d _offset;
      Eigen::Isometry3d _inverseOffset;
  };

  /**
   * \brief caching the offset related to a vertex
   */
  class CacheSE3Offset: public Cache {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      CacheSE3Offset();
      virtual void updateImpl();

      const ParameterSE3Offset* offsetParam() const { return _offsetParam;}
      void setOffsetParam(ParameterSE3Offset* offsetParam);

      const Eigen::Isometry3d& w2n() const { return _w2n;}
      const Eigen::Isometry3d& n2w() const { return _n2w;}
      const Eigen::Isometry3d& w2l() const { return _w2l;}

    protected:
      ParameterSE3Offset* _offsetParam; ///< the parameter connected to the cache
      Eigen::Isometry3d _w2n;
      Eigen::Isometry3d _n2w;
      Eigen::Isometry3d _w2l;

    protected:
      virtual bool resolveDependancies();
  };


#ifdef G2O_HAVE_OPENGL
  class CacheSE3OffsetDrawAction: public DrawAction{
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
