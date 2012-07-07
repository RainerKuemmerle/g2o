#ifndef G2O_TARGET_TYPES_6D_HPP_
#define G2O_TARGET_TYPES_6D_HPP_

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <Eigen/Core>

using namespace g2o;

typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,6,6> Matrix6d;

// This header file specifies a set of types for the different
// tracking examples; note that 

class VertexPosition3D : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexPosition3D()
  {
  }
  
  virtual void setToOriginImpl() {
    _estimate.setZero();
  }
  
  virtual void oplusImpl(const double* update)
  {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    _estimate[2] += update[2];
  }
  
  virtual bool read(std::istream& /*is*/)
  {
    return false;
  }
  
  virtual bool write(std::ostream& /*os*/) const
  {
    return false;
  }
  
};

class PositionVelocity3DEdge
{
};
 
class VertexPositionVelocity3D : public g2o::BaseVertex<6, Vector6d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexPositionVelocity3D()
  {
  }
  
  virtual void setToOriginImpl() {
    _estimate.setZero();
  }
  
  virtual void oplusImpl(const double* update)
  {
    for (int k = 0; k < 6; k++)
      _estimate[k] += update[k];
  }
  

  virtual bool read(std::istream& /*is*/)
  {
    return false;
  }
  
  virtual bool write(std::ostream& /*os*/) const
  {
    return false;
  }
  
};

// The odometry which links pairs of nodes together
class TargetOdometry3DEdge : public g2o::BaseBinaryEdge<6, Eigen::Vector3d, VertexPositionVelocity3D, VertexPositionVelocity3D>
{
public:
  TargetOdometry3DEdge(double dt, double noiseSigma)
  {
    _dt = dt;

    double q = noiseSigma * noiseSigma;
    double dt2 = dt * dt;

    // Process noise covariance matrix; this assumes an "impulse"
    // noise model; we add a small stabilising term on the diagonal to make it invertible
    Matrix6d Q=Matrix6d::Zero();
    Q(0, 0) = Q(1,1) = Q(2,2) = dt2*dt2*q/4 + 1e-4;
    Q(0, 3) = Q(1, 4) = Q(2, 5) = dt*dt2*q/2;
    Q(3, 3) = Q(4,4) = Q(5,5) = dt2 * q + 1e-4;
    Q(3, 0) = Q(4, 1) = Q(5, 2) = dt*dt2*q/2;

    setInformation(Q.inverse());
  }

  /** set the estimate of the to vertex, based on the estimate of the from vertex in the edge. */
  virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* to){
    assert(from.size() == 1);
    const VertexPositionVelocity3D* vi = static_cast<const VertexPositionVelocity3D*>(*from.begin());
    VertexPositionVelocity3D* vj = static_cast<VertexPositionVelocity3D*>(to);
    Vector6d viEst=vi->estimate();
    Vector6d vjEst=viEst;

    for (int m = 0; m < 3; m++)
    {
      vjEst[m] += _dt * (vjEst[m+3] + 0.5 * _dt * _measurement[m]);
    }

    for (int m = 0; m < 3; m++)
    {
      vjEst[m+3] += _dt * _measurement[m];
    }
    vj->setEstimate(vjEst);
  }

  /** override in your class if it's not possible to initialize the vertices in certain combinations */
  virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* to) {
    //only works on sequential vertices
    const VertexPositionVelocity3D* vi = static_cast<const VertexPositionVelocity3D*>(*from.begin());
    return (to->id() - vi->id() == 1) ? 1.0 : -1.0;
  }


  void computeError()
  {
    const VertexPositionVelocity3D* vi = static_cast<const VertexPositionVelocity3D*>(_vertices[0]);
    const VertexPositionVelocity3D* vj = static_cast<const VertexPositionVelocity3D*>(_vertices[1]);
    
    for (int k = 0; k < 3; k++)
      {
        _error[k] = vi->estimate()[k] + _dt * (vi->estimate()[k+3] + 0.5 * _dt * _measurement[k]) - vj->estimate()[k];
      }
    for (int k = 3; k < 6; k++)
      {
        _error[k] = vi->estimate()[k] + _dt * _measurement[k-3]- vj->estimate()[k];
      }
  }
  
  virtual bool read(std::istream& /*is*/)
  {
    return false;
  }
  
  virtual bool write(std::ostream& /*os*/) const
  {
    return false;
  }

private:
  double _dt;
};

// The GPS 
class GPSObservationEdgePositionVelocity3D : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPositionVelocity3D>
{
public:
  GPSObservationEdgePositionVelocity3D(const Eigen::Vector3d& measurement, double noiseSigma)
  {
    setMeasurement(measurement);
    setInformation(Eigen::Matrix3d::Identity() / (noiseSigma*noiseSigma));
  }
  
  void computeError()
  {
    const VertexPositionVelocity3D* v = static_cast<const VertexPositionVelocity3D*>(_vertices[0]);
    for (int k = 0; k < 3; k++)
      {
        _error[k] = v->estimate()[k] - _measurement[k];
      }    
  }
  
  virtual bool read(std::istream& /*is*/)
  {
    return false;
  }
  
  virtual bool write(std::ostream& /*os*/) const
  {
    return false;
  }
};


#endif //  __TARGET_TYPES_6D_HPP__
