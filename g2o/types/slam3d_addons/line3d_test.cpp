#include "line3d.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"
#include "g2o/types/slam3d/isometry3d_gradients.h"
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace g2o;
using namespace g2o::internal;

template <typename T>
ostream& printVector(ostream& os, const T& t)
{
  for (int i=0; i<t.rows(); i++){
    os << t(i) << " ";
  }
  return os;
}

int main(int , char** )
{
  Vector6d t;
  t <<  -3, -2, -4, .2, .1, .3;
  Isometry3D T = fromVectorMQT(t);
  cout << "transform" << endl;
  cout << T.matrix() << endl;

  Vector6d cl1;
  cl1 << 20, 50, -70, .1, .2, .3;
  cl1 =normalizeCartesianLine(cl1);
  cout << "cartesian line L1: "; printVector(cout, cl1); cout << endl;
  Line3D pl1 = Line3D::fromCartesian(cl1);
  cout << "pluecker line L1: "; printVector(cout, pl1); cout << endl;
  cout << "cartesian line L1, reconstructed from puecker: "; printVector(cout, pl1.toCartesian()); cout << endl;

  Vector6d cl2=transformCartesianLine(T,cl1);
  Line3D pl2 = T*pl1;;

  cout << "transformed line L2: "; printVector(cout, cl2); cout << endl;
  cout << "transformed pline L2: "; printVector(cout, pl2); cout << endl;
  cout << "error of cartesian line L2, reconstructed from puecker: "; printVector(cout, cl2 - pl2.toCartesian());
  cout << endl;

  return 0;
}
