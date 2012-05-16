#include <iostream>
#include "isometry3d_mappings.h"

using namespace std;
using namespace g2o;

int main(int , char** ){
  Vector3d eulerAngles(.1,.2,.3);
  Matrix3d m1=fromEuler(eulerAngles);
  cerr << "m1=fromEuler(eulerAngles)" << endl;
  cerr << "eulerAngles:" << endl;
  cerr << eulerAngles << endl;
  cerr << "m1:" << endl;
  cerr << m1 << endl;

  Vector3d eulerAngles1 =  toEuler(m1);
  cerr << "eulerAngles1 =  toEuler(m1) " << endl;
  cerr << "eulerAngles1:" << endl;
  cerr << eulerAngles1 << endl;
  
  Vector3d q=toCompactQuaternion(m1);
  cerr << "q=toCompactQuaternion(m1)" << endl;
  cerr << "q:" << endl;
  cerr <<  q << endl;

  Matrix3d m2=fromCompactQuaternion(q);
  cerr << "m2=fromCompactQuaternion(q);" << endl;
  cerr << "m2:" << endl;
  cerr << m2 << endl;

  Vector6d et;
  Vector3d t(1.,2.,3.);
  et.block<3,1>(0,0)=eulerAngles;
  et.block<3,1>(3,0)=t;
  Isometry3d i1 = fromVectorET(et);
  cerr << "i1 = fromVectorET(et);" << endl;
  cerr << "et:" << endl;
  cerr << et << endl;
  cerr << "i1" << endl;
  cerr << i1.rotation() << endl;
  cerr << i1.translation() << endl;
  Vector6d et2=toVectorET(i1);
  cerr << "et2=toVectorET(i1);" << endl;
  cerr << "et2" << endl;
  cerr << et2 << endl;
  
  Vector6d qt1=toVectorMQT(i1);
  cerr << "qt1=toVectorMQT(i1)" << endl;
  cerr << "qt1:" << endl;
  cerr << qt1 << endl;

  Isometry3d i2 = fromVectorMQT(qt1);
  cerr << "i2 = fromVectorMQT(qt1)" << endl;
  cerr << "i2" << endl;
  cerr << i2.rotation() << endl;
  cerr << i2.translation() << endl;

  Vector7d qt2=toVectorQT(i1);
  cerr << "qt2=toVectorQT(i1)" << endl;
  cerr << "qt2:" << endl;
  cerr << qt2 << endl;

  Isometry3d i3 = fromVectorQT(qt2);
  cerr << "i3 = fromVectorQT(qt2)" << endl;
  cerr << "i3" << endl;
  cerr << i3.rotation() << endl;
  cerr << i3.translation() << endl;

}
