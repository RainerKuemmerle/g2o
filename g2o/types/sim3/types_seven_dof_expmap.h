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


#ifndef G2O_SEVEN_DOF_EXPMAP_TYPES
#define G2O_SEVEN_DOF_EXPMAP_TYPES

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "sim3.h"

namespace g2o {

  namespace types_seven_dof_expmap {
    void init();
  }

  using namespace Eigen;

  /**
 * \brief Sim3 Vertex, (x,y,z,qw,qx,qy,qz)
 * the parameterization for the increments constructed is a 7d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 */
  class VertexSim3Expmap : public BaseVertex<7, Sim3>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexSim3Expmap();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
      _estimate = Sim3();
    }

    virtual void oplusImpl(const double* update_)
    {
      Map<Vector7d> update(const_cast<double*>(update_));

      if (_fix_scale)
        update[6] = 0;

      Sim3 s(update);
      setEstimate(s*estimate());
    }

    Vector2d _principle_point;
    Vector2d _focal_length;

    Vector2d cam_map(const Vector2d & v) const
    {
      Vector2d res;
      res[0] = v[0]*_focal_length[0] + _principle_point[0];
      res[1] = v[1]*_focal_length[1] + _principle_point[1];
      return res;
    }

    bool _fix_scale;


  protected:
  };

  /**
 * \brief 7D edge between two Vertex7
 */
  class EdgeSim3 : public BaseBinaryEdge<7, Sim3, VertexSim3Expmap, VertexSim3Expmap>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    void computeError()
    {
      const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[0]);
      const VertexSim3Expmap* v2 = static_cast<const VertexSim3Expmap*>(_vertices[1]);

      Sim3 C(_measurement);
      Sim3 error_=C*v1->estimate()*v2->estimate().inverse();
      _error = error_.log();
    }

    virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 1.;}
    virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/)
    {
      VertexSim3Expmap* v1 = static_cast<VertexSim3Expmap*>(_vertices[0]);
      VertexSim3Expmap* v2 = static_cast<VertexSim3Expmap*>(_vertices[1]);
      if (from.count(v1) > 0)
  v2->setEstimate(measurement()*v1->estimate());
      else
  v1->setEstimate(measurement().inverse()*v2->estimate());
    }
  };


/**/
class EdgeSim3ProjectXYZ : public  BaseBinaryEdge<2, Vector2d,  VertexSBAPointXYZ, VertexSim3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3ProjectXYZ();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError()
    {
      const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[1]);
      const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

      Vector2d obs(_measurement);
      _error = obs-v1->cam_map(project(v1->estimate().map(v2->estimate())));
    }

   // virtual void linearizeOplus();

};

} // end namespace

#endif
