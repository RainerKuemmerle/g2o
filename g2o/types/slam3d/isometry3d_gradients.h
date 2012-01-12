#ifndef G2O_ISOMETRY3D_GRADIENTS_H
#define G2O_ISOMETRY3D_GRADIENTS_H_
#include <Eigen/Geometry>
namespace g2o {
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  void computeEdgeSE3Gradient(Eigen::Isometry3d& E,
                              Matrix6d& Ji, 
                              Matrix6d& Jj,
                              const Eigen::Isometry3d& Z, 
                              const Eigen::Isometry3d& Xi,
                              const Eigen::Isometry3d& Xj,
                              const Eigen::Isometry3d& Pi=Eigen::Isometry3d(), 
                              const Eigen::Isometry3d& Pj=Eigen::Isometry3d());

void computeEdgeSE3PriorGradient(Eigen::Isometry3d& E,
                                 Matrix6d& J, 
                                 const Eigen::Isometry3d& Z, 
                                 const Eigen::Isometry3d& X,
                                 const Eigen::Isometry3d& P=Eigen::Isometry3d());

}
#endif
