// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
//
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef G2O_SIX_DOF_TYPES_EXPMAP
#define G2O_SIX_DOF_TYPES_EXPMAP

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/types/slam3d/se3_ops.h"
#include "types_sba.h"
#include <Eigen/Geometry>

namespace g2o {
  namespace types_six_dof_expmap {
    void init();
  }

  using namespace Eigen;

  typedef Matrix<double, 6, 6> Matrix6d;

/**
 * \brief SE3 Vertex parameterized internally with a transformation matrix
 and externally with its exponential map 
 */
  class VertexSE3Expmap : public BaseVertex<6, SE3Quat>
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      VertexSE3Expmap();
      bool read(std::istream& is);
      bool write(std::ostream& os) const;

      virtual void setToOriginImpl() {
        _estimate = SE3Quat();
      }
      
      virtual void oplusImpl(const double* update_)
      {
        Map<const Vector6d> update(update_);

        //SE3 res = s*SE3(estimate());
        setEstimate(SE3Quat::exp(update)*estimate());

        //SE3Quat(Quaterniond(res._R),
        //             res.translation());
      }

      Vector2d _principle_point;
      Vector2d _focal_length;
      double _baseline;


      Vector2d cam_map(const Vector3d & trans_xyz) const
      {
        Vector2d proj = project(trans_xyz);

        Vector2d res;
        res[0] = proj[0]*_focal_length[0] + _principle_point[0];
        res[1] = proj[1]*_focal_length[1] + _principle_point[1];
        return res;
      }


      Vector3d stereocam_uvq_map(const Vector3d & trans_xyz) const
      {
        Vector2d uv_left = cam_map(trans_xyz);

        double proj_x_right = (trans_xyz[0]-_baseline)/trans_xyz[2];
        double u_right = proj_x_right*_focal_length[0] + _principle_point[0];

        Vector3d res;
        res[0] = uv_left[0];
        res[1] = uv_left[1];
        res[2] = (uv_left[0]-u_right)/_baseline;
        return res;
      }

      Vector3d stereocam_uvu_map(const Vector3d & trans_xyz) const
      {
        Vector2d uv_left = cam_map(trans_xyz);

        double proj_x_right = (trans_xyz[0]-_baseline)/trans_xyz[2];
        double u_right = proj_x_right*_focal_length[0] + _principle_point[0];


        return Vector3d(uv_left[0],uv_left[1],u_right);
      }

};


/**
 * \brief 6D edge between two Vertex6
 */
class EdgeSE3Expmap : public BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>
{
  // no chain rule, numeric differentiation on linearizeOplus
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3Expmap();
    bool read(std::istream& is);
    bool write(std::ostream& os) const;
    void computeError()
    {
      const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
      const VertexSE3Expmap* v2 = static_cast<const VertexSE3Expmap*>(_vertices[1]);

      SE3Quat C(_measurement);
      SE3Quat error_= v2->estimate().inverse()*C*v1->estimate();
      _error = error_.log();
    }

    virtual void linearizeOplus();

};


class EdgeProjectXYZ2UV : public  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeProjectXYZ2UV();
    bool read(std::istream& is);
    bool write(std::ostream& os) const;

    void computeError()
    {
      const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
      const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

      Vector2d obs(_measurement);
      _error = obs-v1->cam_map(v1->estimate().map(v2->estimate()));
    }

    virtual void linearizeOplus();
};


class EdgeProjectXYZ2UVQ : public  BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeProjectXYZ2UVQ();
    bool read(std::istream& is);
    bool write(std::ostream& os) const;

    void computeError()
    {
      const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
      const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

      Vector3d obs(_measurement);
      _error = obs-v1->stereocam_uvq_map(v1->estimate().map(v2->estimate()));
    }

    virtual void linearizeOplus();

};

//Stereo Observations
// U: left u
// V: left v
// U: right u
class EdgeProjectXYZ2UVU : public  BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeProjectXYZ2UVU();

    bool read(std::istream& is);
    bool write(std::ostream& os) const;

    void computeError()
    {
      const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
      const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

      Vector3d obs(_measurement);
      _error = obs-v1->stereocam_uvu_map(v1->estimate().map(v2->estimate()));
    }

    //virtual void linearizeOplus();

};

} // end namespace

#endif
