#include <iostream>

#include "se3quat.h"
using namespace std;
using namespace g2o;

int main()
{
  Vector7d v7;
  v7 << 1,2,3,0.5,0.5,0.5,0.5;
  SE3Quat p1(v7);
  Vector6d minimalVector = p1.toMinimalVector();
  cerr << minimalVector.transpose() << endl;

  SE3Quat p2;
  p2.fromMinimalVector(minimalVector);
  cerr << p2.rotation().coeffs().transpose() << endl;
}
