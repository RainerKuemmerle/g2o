// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "types_six_dof_expmap.h"

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

namespace g2o {

using namespace std;
using namespace Eigen;

G2O_REGISTER_TYPE_GROUP(expmap);
G2O_REGISTER_TYPE(VERTEX_SE3:EXPMAP, VertexSE3Expmap);
G2O_REGISTER_TYPE(EDGE_SE3:EXPMAP, EdgeSE3Expmap);
G2O_REGISTER_TYPE(EDGE_PROJECT_XYZ2UV:EXPMAP, EdgeProjectXYZ2UV);
G2O_REGISTER_TYPE(EDGE_PROJECT_XYZ2UVU:EXPMAP, EdgeProjectXYZ2UVU);
G2O_REGISTER_TYPE(PARAMS_CAMERAPARAMETERS, CameraParameters);

CameraParameters
::CameraParameters()
  : focal_length(1.),
    principle_point(Vector2D(0., 0.)),
    baseline(0.5)  {
}

Vector2D project2d(const Vector3D& v)  {
  Vector2D res;
  res(0) = v(0)/v(2);
  res(1) = v(1)/v(2);
  return res;
}

Vector3D unproject2d(const Vector2D& v)  {
  Vector3D res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = 1;
  return res;
}

inline Vector3D invert_depth(const Vector3D & x){
  return unproject2d(x.head<2>())/x[2];
}

Vector2D  CameraParameters::cam_map(const Vector3D & trans_xyz) const {
  Vector2D proj = project2d(trans_xyz);
  Vector2D res;
  res[0] = proj[0]*focal_length + principle_point[0];
  res[1] = proj[1]*focal_length + principle_point[1];
  return res;
}

Vector3D CameraParameters::stereocam_uvu_map(const Vector3D & trans_xyz) const {
  Vector2D uv_left = cam_map(trans_xyz);
  double proj_x_right = (trans_xyz[0]-baseline)/trans_xyz[2];
  double u_right = proj_x_right*focal_length + principle_point[0];
  return Vector3D(uv_left[0],uv_left[1],u_right);
}


VertexSE3Expmap::VertexSE3Expmap() : BaseVertex<6, SE3Quat>() {
}

bool VertexSE3Expmap::read(std::istream& is) {
  Vector7d est;
  for (int i=0; i<7; i++)
    is  >> est[i];
  SE3Quat cam2world;
  cam2world.fromVector(est);
  setEstimate(cam2world.inverse());
  return true;
}

bool VertexSE3Expmap::write(std::ostream& os) const {
  SE3Quat cam2world(estimate().inverse());
  for (int i=0; i<7; i++)
    os << cam2world[i] << " ";
  return os.good();
}

EdgeSE3Expmap::EdgeSE3Expmap() :
  BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>() {
}

bool EdgeSE3Expmap::read(std::istream& is)  {
  Vector7d meas;
  for (int i=0; i<7; i++)
    is >> meas[i];
  SE3Quat cam2world;
  cam2world.fromVector(meas);
  setMeasurement(cam2world.inverse());
  //TODO: Convert information matrix!!
  for (int i=0; i<6; i++)
    for (int j=i; j<6; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3Expmap::write(std::ostream& os) const {
  SE3Quat cam2world(measurement().inverse());
  for (int i=0; i<7; i++)
    os << cam2world[i] << " ";
  for (int i=0; i<6; i++)
    for (int j=i; j<6; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

EdgeProjectXYZ2UV::EdgeProjectXYZ2UV() : BaseBinaryEdge<2, Vector2D, VertexSBAPointXYZ, VertexSE3Expmap>() {
  _cam = 0;
  resizeParameters(1);
  installParameter(_cam, 0);
}

bool EdgeProjectPSI2UV::write(std::ostream& os) const  {
  os << _cam->id() << " ";
  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

bool EdgeProjectPSI2UV::read(std::istream& is) {
  int paramId;
  is >> paramId;
  setParameterId(0, paramId);

  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

void EdgeProjectPSI2UV::computeError(){
  const VertexSBAPointXYZ * psi = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
  const VertexSE3Expmap * T_p_from_world = static_cast<const VertexSE3Expmap*>(_vertices[1]);
  const VertexSE3Expmap * T_anchor_from_world = static_cast<const VertexSE3Expmap*>(_vertices[2]);
  const CameraParameters * cam = static_cast<const CameraParameters *>(parameter(0));

  Vector2D obs(_measurement);
  _error = obs - cam->cam_map(T_p_from_world->estimate()
        *T_anchor_from_world->estimate().inverse()
        *invert_depth(psi->estimate()));
}

inline Matrix<double,2,3,Eigen::ColMajor> d_proj_d_y(const double & f, const Vector3D & xyz){
  double z_sq = xyz[2]*xyz[2];
  Matrix<double,2,3,Eigen::ColMajor> J;
  J << f/xyz[2], 0,           -(f*xyz[0])/z_sq,
      0,           f/xyz[2], -(f*xyz[1])/z_sq;
  return J;
}

inline Matrix<double,3,6,Eigen::ColMajor> d_expy_d_y(const Vector3D & y){
  Matrix<double,3,6,Eigen::ColMajor> J;
  J.topLeftCorner<3,3>() = -skew(y);
  J.bottomRightCorner<3,3>().setIdentity();

  return J;
}

inline Matrix3D d_Tinvpsi_d_psi(const SE3Quat & T, const Vector3D & psi){
  Matrix3D R = T.rotation().toRotationMatrix();
  Vector3D x = invert_depth(psi);
  Vector3D r1 = R.col(0);
  Vector3D r2 = R.col(1);
  Matrix3D J;
  J.col(0) = r1;
  J.col(1) = r2;
  J.col(2) = -R*x;
  J*=1./psi.z();
  return J;
}

void EdgeProjectPSI2UV::linearizeOplus(){
  VertexSBAPointXYZ* vpoint = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
  Vector3D psi_a = vpoint->estimate();
  VertexSE3Expmap * vpose = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T_cw = vpose->estimate();
  VertexSE3Expmap * vanchor = static_cast<VertexSE3Expmap *>(_vertices[2]);
  const CameraParameters * cam
      = static_cast<const CameraParameters *>(parameter(0));

  SE3Quat A_aw = vanchor->estimate();
  SE3Quat T_ca = T_cw*A_aw.inverse();
  Vector3D x_a = invert_depth(psi_a);
  Vector3D y = T_ca*x_a;
  Matrix<double,2,3,Eigen::ColMajor> Jcam
      = d_proj_d_y(cam->focal_length, y);
  _jacobianOplus[0] = -Jcam*d_Tinvpsi_d_psi(T_ca, psi_a);
  _jacobianOplus[1] = -Jcam*d_expy_d_y(y);
  _jacobianOplus[2] = Jcam*T_ca.rotation().toRotationMatrix()*d_expy_d_y(x_a);
}



EdgeProjectXYZ2UVU::EdgeProjectXYZ2UVU() : BaseBinaryEdge<3, Vector3D, VertexSBAPointXYZ, VertexSE3Expmap>()
{
  _cam = 0;
  resizeParameters(1);
  installParameter(_cam, 0);
}

bool EdgeProjectXYZ2UV::read(std::istream& is){
  int paramId;
  is >> paramId;
  setParameterId(0, paramId);

  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeProjectXYZ2UV::write(std::ostream& os) const {
  os << _cam->id() << " ";
  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

void EdgeSE3Expmap::linearizeOplus() {
  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  SE3Quat Ti(vi->estimate());

  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat Tj(vj->estimate());

  const SE3Quat & Tij = _measurement;
  SE3Quat invTij = Tij.inverse();

  SE3Quat invTj_Tij = Tj.inverse()*Tij;
  SE3Quat infTi_invTij = Ti.inverse()*invTij;

  _jacobianOplusXi = invTj_Tij.adj();
  _jacobianOplusXj = -infTi_invTij.adj();
}

void EdgeProjectXYZ2UV::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate());
  VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
  Vector3D xyz = vi->estimate();
  Vector3D xyz_trans = T.map(xyz);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z*z;

  const CameraParameters * cam = static_cast<const CameraParameters *>(parameter(0));

  Matrix<double,2,3,Eigen::ColMajor> tmp;
  tmp(0,0) = cam->focal_length;
  tmp(0,1) = 0;
  tmp(0,2) = -x/z*cam->focal_length;

  tmp(1,0) = 0;
  tmp(1,1) = cam->focal_length;
  tmp(1,2) = -y/z*cam->focal_length;

  _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();

  _jacobianOplusXj(0,0) =  x*y/z_2 *cam->focal_length;
  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *cam->focal_length;
  _jacobianOplusXj(0,2) = y/z *cam->focal_length;
  _jacobianOplusXj(0,3) = -1./z *cam->focal_length;
  _jacobianOplusXj(0,4) = 0;
  _jacobianOplusXj(0,5) = x/z_2 *cam->focal_length;

  _jacobianOplusXj(1,0) = (1+y*y/z_2) *cam->focal_length;
  _jacobianOplusXj(1,1) = -x*y/z_2 *cam->focal_length;
  _jacobianOplusXj(1,2) = -x/z *cam->focal_length;
  _jacobianOplusXj(1,3) = 0;
  _jacobianOplusXj(1,4) = -1./z *cam->focal_length;
  _jacobianOplusXj(1,5) = y/z_2 *cam->focal_length;
}

bool EdgeProjectXYZ2UVU::read(std::istream& is){
  for (int i=0; i<3; i++){
    is  >> _measurement[i];
  }
  for (int i=0; i<3; i++)
    for (int j=i; j<3; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeProjectXYZ2UVU::write(std::ostream& os) const {
  for (int i=0; i<3; i++){
    os  << measurement()[i] << " ";
  }

  for (int i=0; i<3; i++)
    for (int j=i; j<3; j++){
      os << " " << information()(i,j);
    }
  return os.good();
}

} // end namespace
