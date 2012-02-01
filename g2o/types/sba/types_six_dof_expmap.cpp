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

  G2O_REGISTER_TYPE_GROUP(expmap);

  G2O_REGISTER_TYPE(VERTEX_SE3:EXPMAP, VertexSE3Expmap);
  G2O_REGISTER_TYPE(EDGE_SE3:EXPMAP, EdgeSE3Expmap);
  G2O_REGISTER_TYPE(EDGE_PROJECT_XYZ2UV:EXPMAP, EdgeProjectXYZ2UV);
  G2O_REGISTER_TYPE(EDGE_PROJECT_XYZ2UVQ:EXPMAP, EdgeProjectXYZ2UVQ);
  G2O_REGISTER_TYPE(EDGE_PROJECT_XYZ2UVU:EXPMAP, EdgeProjectXYZ2UVU);

  VertexSE3Expmap::VertexSE3Expmap() : BaseVertex<6, SE3Quat>()
  {
    _principle_point[0] = 0;
    _principle_point[1] = 0;
    _focal_length[0] = 1;
    _focal_length[1] = 1;
    _baseline = 0.5;
  }

  bool VertexSE3Expmap::read(std::istream& is)
  {
    Vector7d est;
    for (int i=0; i<7; i++)
      is  >> est[i];
    SE3Quat cam2world;
    cam2world.fromVector(est);
    setEstimate(cam2world.inverse());
    for (int i=0; i<2; i++)
      is >> _focal_length[i];
    for (int i=0; i<2; i++)
      is >> _principle_point[i];
    is >> _baseline;
    return true;
  }

  bool VertexSE3Expmap::write(std::ostream& os) const
  {
    SE3Quat cam2world(estimate().inverse());
    for (int i=0; i<7; i++)
      os << cam2world[i] << " ";
    for (int i=0; i<2; i++)
      os << _focal_length[i] << " ";
    for (int i=0; i<2; i++)
      os << _principle_point[i] << " ";
    os << _baseline << " ";
    return os.good();
  }

  EdgeSE3Expmap::EdgeSE3Expmap() :
    BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>()
  {
  }

  bool EdgeSE3Expmap::read(std::istream& is)
  {
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

  bool EdgeSE3Expmap::write(std::ostream& os) const
  {
    SE3Quat cam2world(measurement().inverse());
    for (int i=0; i<7; i++)
      os << cam2world[i] << " ";
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++){
        os << " " <<  information()(i,j);
      }
    return os.good();
  }

  EdgeProjectXYZ2UV::EdgeProjectXYZ2UV() :
  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>()
  {
  }

  EdgeProjectXYZ2UVQ::EdgeProjectXYZ2UVQ() :
  BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>()
  {
  }

  EdgeProjectXYZ2UVU::EdgeProjectXYZ2UVU() :
  BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>()
  {
  }

  bool EdgeProjectXYZ2UV::read(std::istream& is)
  {
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

  bool EdgeProjectXYZ2UV::write(std::ostream& os) const
  {
    for (int i=0; i<2; i++){
      os << measurement()[i] << " ";
    }

    for (int i=0; i<2; i++)
      for (int j=i; j<2; j++){
        os << " " <<  information()(i,j);
      }
    return os.good();
  }

  void EdgeSE3Expmap::linearizeOplus()
  {
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

  void EdgeProjectXYZ2UV::linearizeOplus()
  {

    VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
    SE3Quat T(vj->estimate());

    VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
    Vector3d xyz = vi->estimate();
    Vector3d xyz_trans = T.map(xyz);

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double z_2 = z*z;

    Matrix<double,2,3> tmp;
    tmp(0,0) = vj->_focal_length(0);
    tmp(0,1) = 0;
    tmp(0,2) = -x/z*vj->_focal_length(0);

    tmp(1,0) = 0;
    tmp(1,1) = vj->_focal_length(1);
    tmp(1,2) = -y/z*vj->_focal_length(1);

    _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();

    _jacobianOplusXj(0,0) =  x*y/z_2 *vj->_focal_length(0);
    _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *vj->_focal_length(0);
    _jacobianOplusXj(0,2) = y/z *vj->_focal_length(0);
    _jacobianOplusXj(0,3) = -1./z *vj->_focal_length(0);
    _jacobianOplusXj(0,4) = 0;
    _jacobianOplusXj(0,5) = x/z_2 *vj->_focal_length(0);


    _jacobianOplusXj(1,0) = (1+y*y/z_2) *vj->_focal_length(1);
    _jacobianOplusXj(1,1) = -x*y/z_2 *vj->_focal_length(1);
    _jacobianOplusXj(1,2) = -x/z *vj->_focal_length(1);
    _jacobianOplusXj(1,3) = 0;
    _jacobianOplusXj(1,4) = -1./z *vj->_focal_length(1);
    _jacobianOplusXj(1,5) = y/z_2 *vj->_focal_length(1);
  }


  void EdgeProjectXYZ2UVQ::linearizeOplus()
  {

    VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
    SE3Quat T(vj->estimate());

    VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
    Vector3d xyz = vi->estimate();
    Vector3d xyz_trans = T.map(xyz);

    const double & x = xyz_trans[0];
    const double & y = xyz_trans[1];
    const double & z = xyz_trans[2];
    double z_sq = z*z;
    const double & Fx = vj->_focal_length(0);
    const double & Fy = vj->_focal_length(1);
    double dq_dz = -Fx/z_sq;
    double x_Fx_by_zsq = x*Fx/z_sq;
    double y_Fy_by_zsq = y*Fy/z_sq;


     Matrix3d R = T.rotation().toRotationMatrix();
    _jacobianOplusXi.row(0) = -Fx/z*R.row(0) + x_Fx_by_zsq*R.row(2);
    _jacobianOplusXi.row(1) = -Fy/z*R.row(1) + y_Fy_by_zsq*R.row(2);
    _jacobianOplusXi.row(2) =  -dq_dz*R.row(2);


    _jacobianOplusXj(0,0) =  x*y/z_sq *Fx;
    _jacobianOplusXj(0,1) = -(1+(x*x/z_sq)) *Fx;
    _jacobianOplusXj(0,2) = y/z *Fx;
    _jacobianOplusXj(0,3) = -1./z *Fx;
    _jacobianOplusXj(0,4) = 0;
    _jacobianOplusXj(0,5) = x/z_sq *Fx;


    _jacobianOplusXj(1,0) = (1+y*y/z_sq) *Fy;
    _jacobianOplusXj(1,1) = -x*y/z_sq *Fy;
    _jacobianOplusXj(1,2) = -x/z *Fy;
    _jacobianOplusXj(1,3) = 0;
    _jacobianOplusXj(1,4) = -1./z *Fy;
    _jacobianOplusXj(1,5) = y/z_sq *Fy;

    _jacobianOplusXj(2,0) = -y*dq_dz ;
    _jacobianOplusXj(2,1) = x*dq_dz;
    _jacobianOplusXj(2,2) = 0;
    _jacobianOplusXj(2,3) = 0;
    _jacobianOplusXj(2,4) = 0;
    _jacobianOplusXj(2,5) = -dq_dz ;

//    std::cerr << _jacobianOplusXi << std::endl;
//    std::cerr << _jacobianOplusXj << std::endl;

//    BaseBinaryEdge<3, Vector3d, VertexPointXYZ, VertexSE3Expmap, false>::linearizeOplus();
//    std::cerr << _jacobianOplusXi << std::endl;
//    std::cerr << _jacobianOplusXj << std::endl;
  }

  bool EdgeProjectXYZ2UVQ::read(std::istream& is)
  {
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

  bool EdgeProjectXYZ2UVQ::write(std::ostream& os) const
  {
    for (int i=0; i<3; i++){
      os  << measurement()[i] << " ";
    }

    for (int i=0; i<3; i++)
      for (int j=i; j<3; j++){
        os << " " <<  information()(i,j);
    }
    return os.good();
  }


  bool EdgeProjectXYZ2UVU::read(std::istream& is)
  {
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

  bool EdgeProjectXYZ2UVU::write(std::ostream& os) const
  {
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
