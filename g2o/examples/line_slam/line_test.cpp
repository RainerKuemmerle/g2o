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

ostream& printPlueckerLine(ostream& os, const Line3D& l) {
  Eigen::Vector3d direction = l.d();
  os << "Direction = ";
  printVector(os, direction);
  os << std::endl;
  Eigen::Vector3d moment = l.w();
  os << "Moment = ";
  printVector(os, moment);
  os << std::endl;
  return os;
}

int main(int /*argc*/, char** /*argv*/) {
  Line3D l0;
  std::cout << "l0: " << std::endl;
  printPlueckerLine(std::cout, l0);
  std::cout << std::endl;

  Vector6d v;
  v << 1.0, 1.0, -0.3, 0.5, 0.2, 0.3;
  Line3D l1(v);
  l1.normalize();
  std::cout << "v: ";
  printVector(std::cout, v);
  std::cout << std::endl;
  std::cout << "l1: " << std::endl;
  printPlueckerLine(std::cout, l1);
  std::cout << "azimuth: " << g2o::internal::getAzimuth(l1.d()) << std::endl;
  std::cout << "elevation: " << g2o::internal::getElevation(l1.d()) << std::endl;
  std::cout << std::endl;

  Line3D l2(l1);
  std::cout << "l2: " << std::endl;
  printPlueckerLine(std::cout, l2);
  std::cout << std::endl;

  Eigen::Vector4d mv = l2.ominus(l1);
  Eigen::Quaterniond q(sqrt(1 - mv.head<3>().squaredNorm()), mv.x(), mv.y(), mv.z());
  Eigen::Vector3d euler_angles = q.toRotationMatrix().eulerAngles(2, 1, 0);
  double yaw = euler_angles[0];
  double pitch = euler_angles[1];
  double roll = euler_angles[2];  
  std::cout << "l1 ominus l2: " << roll << " " << pitch << " " << yaw << " " << mv[3] << std::endl;
  std::cout << std::endl;

  v << 0.0, 0.0, 1.0, 0.5, 0.5, 0.0;
  l1 = Line3D(v);
  l1.normalize();
  std::cout << "l1: " << std::endl;
  printPlueckerLine(std::cout, l1);
  std::cout << "azimuth: " << g2o::internal::getAzimuth(l1.d()) << std::endl;
  std::cout << "elevation: " << g2o::internal::getElevation(l1.d()) << std::endl;
  std::cout << std::endl;
  
  v << 0.0, 0.0, 1.0, 0.5, -0.5, 0.0;
  l2 = Line3D(v);
  l2.normalize();
  std::cout << "l2: " << std::endl;
  printPlueckerLine(std::cout, l2);
  std::cout << "azimuth: " << g2o::internal::getAzimuth(l2.d()) << std::endl;
  std::cout << "elevation: " << g2o::internal::getElevation(l2.d()) << std::endl;
  std::cout << std::endl;

  mv = l2.ominus(l1);
  q = Eigen::Quaterniond(sqrt(1 - mv.head<3>().squaredNorm()), mv.x(), mv.y(), mv.z());
  euler_angles = q.toRotationMatrix().eulerAngles(2, 1, 0);
  yaw = euler_angles[0];
  pitch = euler_angles[1];
  roll = euler_angles[2];  
  std::cout << "l1 ominus l2: " << roll << " " << pitch << " " << yaw << " " << mv[3] << std::endl;
  std::cout << std::endl;
  
  Line3D l3 = Line3D(l1);
  std::cout << "l3: " << std::endl;
  printPlueckerLine(std::cout, l3);
  l3.oplus(mv);
  std::cout << "l3 oplus mv: " << std::endl;
  printPlueckerLine(std::cout, l3);
  std::cout << std::endl;

  std::vector<Line3D> l;
  v << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0;
  Line3D ll = Line3D(v);
  ll.normalize();
  l.push_back(ll);
  v << 0.0, 0.0, 1.0, 0.0, 1.0, 0.0;
  ll = Line3D(v);
  ll.normalize();
  l.push_back(ll);
  v << 0.0, 0.0, 1.0, 0.0, 0.0, 1.0;
  ll = Line3D(v);
  ll.normalize();
  l.push_back(ll);

  for(size_t i = 0; i < l.size(); ++i) {
    Line3D& line = l[i];
    std::cout << "line: "
	      << line.d()[0] << " " << line.d()[1] << " "  << line.d()[2] << " "
	      << line.w()[0] << " " << line.w()[1] << " "  << line.w()[2] << std::endl;
  }
  std::cout << std::endl;

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation().x() = 0.9;
  std::cout << "R: " << std::endl << T.linear() << std::endl;
  std::cout << "t: " << std::endl << T.translation() << std::endl;
  std::cout << std::endl;
  
  for(size_t i = 0; i < l.size(); ++i) {
    Line3D& line = l[i];
    line = T * line;
    std::cout << "line: "
	      << line.d()[0] << " " << line.d()[1] << " " << line.d()[2] << " "
	      << line.w()[0] << " " << line.w()[1] << " " << line.w()[2] << std::endl;
  }
  std::cout << std::endl;
  
  return 0;
}
