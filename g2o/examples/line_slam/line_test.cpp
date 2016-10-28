#include <iostream>

#include "g2o/types/slam3d/isometry3d_mappings.h"
#include "g2o/types/slam3d/isometry3d_gradients.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"

using namespace std;
using namespace Eigen;
using namespace g2o;
using namespace g2o::internal;

template <typename T>
ostream& printVector(ostream& os, const T& t) {
  for(int i = 0; i < t.rows(); ++i) {
    os << t(i) << " ";
  }
  return os;
}

// TODO Jacopo

int main(int /*argc*/, char** /*argv*/) {
  // Vector6d t;
  // t <<  -3.0, -2.0, -4.0, 0.2, 0.1, 0.3;
  // Isometry3D T = fromVectorMQT(t);
  // std::cout << "Transform" << std::endl;
  // std::cout << T.matrix() << std::endl;

  // Vector6d cl1;
  // cl1 << 20.0, 50.0, -70.0, 0.1, 0.2, 0.3;
  // cl1 = normalizeCartesianLine(cl1);
  // std::cout << "Cartesian line L1: ";
  // printVector(std::cout, cl1);
  // std::cout << endl;

  // Line3D pl1 = Line3D::fromCartesian(cl1);
  // std::cout << "Pluecker line L1: ";
  // printVector(std::cout, pl1);
  // std::cout << std::endl;
  // std::cout << "Cartesian line L1, reconstructed from pluecker: ";
  // printVector(std::cout, pl1.toCartesian());
  // std::cout << std::endl;

  // Vector6d cl2 = transformCartesianLine(T, cl1);
  // Line3D pl2 = T*pl1;

  // std::cout << "Transformed line L2: ";
  // printVector(std::cout, cl2);
  // std::cout << std::endl;
  // std::cout << "Transformed pline L2: ";
  // printVector(std::cout, pl2);
  // std::cout << std::endl;
  // std::cout << "Error of cartesian line L2, reconstructed from puecker: ";
  // printVector(std::cout, cl2 - pl2.toCartesian());
  // std::cout << endl;

  return 0;
}
