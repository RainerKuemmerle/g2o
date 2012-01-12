// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
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

///
/// Basic types definitions for SBA translation to HChol
///
/// Camera nodes use camera pose in real world
///   v3 position
///   normalized quaternion rotation
///
/// Point nodes:
///   v3 position
///

#ifndef G2O_SBACam_
#define G2O_SBACam_

#include "g2o/stuff/misc.h"
#include "g2o/stuff/macros.h"
#include "g2o/types/slam3d/se3quat.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

// this seems to have to go outside of the AISNav namespace...
//USING_PART_OF_NAMESPACE_EIGEN;

namespace g2o {
  using namespace Eigen;
  typedef  Matrix<double, 6, 1> Vector6d;
  
  // useful types
  //  typedef  Matrix<double, 6, 6> Matrix6d;

  class SBACam: public SE3Quat
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   public:
    // camera matrix and stereo baseline
    Matrix3d Kcam; 
    double baseline;

    // transformations
    Matrix<double,3,4> w2n; // transform from world to node coordinates
    Matrix<double,3,4> w2i; // transform from world to image coordinates

    // Derivatives of the rotation matrix transpose wrt quaternion xyz, used for
    // calculating Jacobian wrt pose of a projection.
    Matrix3d dRdx, dRdy, dRdz;

    // initialize an object
  SBACam()
    {
      SE3Quat();
      setKcam(1,1,0.5,0.5,0);  // unit image projection
    }


    // set the object pose
  SBACam(const Quaterniond&  r_, const Vector3d& t_) : SE3Quat(r_, t_)
    {
      setTransform();
      setProjection();
      setDr();
    }

  SBACam(const SE3Quat& p) : SE3Quat(p) {
    setTransform();
    setProjection();
    setDr();
  }

    // update from the linear solution
    //defined in se3quat
    void update(const Vector6d& update)
    {
      //      std::cout << "UPDATE " << update.transpose() << std::endl;
      // position update
      _t += update.head(3);
      // small quaternion update
      Quaterniond qr;
      qr.vec() = update.segment<3>(3); 
      qr.w() = sqrt(1.0 - qr.vec().squaredNorm()); // should always be positive
      _r *= qr;                 // post-multiply
      _r.normalize();    
      setTransform();
      setProjection();
      setDr();
      //      std::cout << t.transpose() << std::endl;
      //      std::cout << r.coeffs().transpose() << std::endl;
    }

    // transforms
    static void transformW2F(Matrix<double,3,4> &m, 
           const Vector3d &trans, 
           const Quaterniond &qrot)
    {
      m.block<3,3>(0,0) = qrot.toRotationMatrix().transpose();
      m.col(3).setZero();         // make sure there's no translation
      Vector4d tt;
      tt.head(3) = trans;
      tt[3] = 1.0;
      m.col(3) = -m*tt;
    }

    static void transformF2W(Matrix<double,3,4> &m, 
           const Vector3d &trans, 
           const Quaterniond &qrot)
    {
      m.block<3,3>(0,0) = qrot.toRotationMatrix();
      m.col(3) = trans;
    }

    // set up camera matrix
    void setKcam(double fx, double fy, double cx, double cy, double tx)
    { 
      Kcam.setZero();
      Kcam(0,0) = fx;
      Kcam(1,1) = fy;
      Kcam(0,2) = cx;
      Kcam(1,2) = cy;
      Kcam(2,2) = 1.0;
      baseline = tx;
      setProjection();
      setDr();
    }

    // set transform from world to cam coords
    void setTransform()  { transformW2F(w2n,_t,_r); }

    // Set up world-to-image projection matrix (w2i), assumes camera parameters
    // are filled.
    void setProjection() { w2i = Kcam * w2n; }

    // sets angle derivatives
    void setDr()
    {
      // inefficient, just for testing
      // use simple multiplications and additions for production code in calculating dRdx,y,z
      Matrix3d dRidx, dRidy, dRidz;
      dRidx << 0.0,0.0,0.0,  
  0.0,0.0,2.0,
  0.0,-2.0,0.0;
      dRidy  << 0.0,0.0,-2.0,
  0.0,0.0,0.0,
  2.0,0.0,0.0;
      dRidz  << 0.0,2.0,0.0,  
  -2.0,0.0,0.0,
  0.0,0.0,0.0;

      // for dS'*R', with dS the incremental change
      dRdx = dRidx * w2n.block<3,3>(0,0);
      dRdy = dRidy * w2n.block<3,3>(0,0);
      dRdz = dRidz * w2n.block<3,3>(0,0);
    }

  };


  // human-readable SBACam object
  inline std::ostream& operator <<(std::ostream& out_str,
                                   const SBACam& cam)
  {
    out_str << cam.translation().transpose() << std::endl;
    out_str << cam.rotation().coeffs().transpose() << std::endl << std::endl;
    out_str << cam.Kcam << std::endl << std::endl;
    out_str << cam.w2n << std::endl << std::endl;
    out_str << cam.w2i << std::endl << std::endl;

    return out_str;
  };


  //
  // class for edges from vps to points with normals
  //

  class EdgeNormal
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   public:
    // point position
    Vector3d pos;

    // unit normal
    Vector3d normal;

    // rotation matrix for normal
    Matrix3d R; 

    // initialize an object
    EdgeNormal()
      {
        pos.setZero();
        normal << 0, 0, 1;
        makeRot();
      }

    // set up rotation matrix
    void makeRot()
    {
      Vector3d y;
      y << 0, 1, 0;
      R.row(2) = normal;
      y = y - normal(1)*normal;
      y.normalize();            // need to check if y is close to 0
      R.row(1) = y;
      R.row(0) = normal.cross(R.row(1));
      //      cout << normal.transpose() << endl;
      //      cout << R << endl << endl;
      //      cout << R*R.transpose() << endl << endl;
    }

  };

} // end namespace


#endif  // SBACam
