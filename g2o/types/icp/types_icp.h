// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
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

#ifndef G2O_TYPES_ICP
#define G2O_TYPES_ICP

#define GICP_ANALYTIC_JACOBIANS
//#define SCAM_ANALYTIC_JACOBIANS

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/types/sba/types_sba.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o_types_icp_api.h"

#include <Eigen/Geometry>
#include <iostream>

namespace g2o {

  namespace types_icp {
    void init();
  }
//
// GICP-type edges
// Each measurement is between two rigid points on each 6DOF vertex
//

  //
  // class for edges between two points rigidly attached to vertices
  //

  class G2O_TYPES_ICP_API EdgeGICP
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

   public:
    // point positions
    Vector3 pos0, pos1;

    // unit normals
    Vector3 normal0, normal1;

    // rotation matrix for normal
    Matrix3 R0,R1;

    // initialize an object
    EdgeGICP()
      {
        pos0.setZero();
        pos1.setZero();
        normal0 << 0, 0, 1;
        normal1 << 0, 0, 1;
        //makeRot();
        R0.setIdentity();
        R1.setIdentity();
      }

    // set up rotation matrix for pos0
    void makeRot0() 
    {
      Vector3 y;
      y << 0, 1, 0;
      R0.row(2) = normal0;
      y = y - normal0(1)*normal0;
      y.normalize();            // need to check if y is close to 0
      R0.row(1) = y;
      R0.row(0) = normal0.cross(R0.row(1));
      //      cout << normal.transpose() << endl;
      //      cout << R0 << endl << endl;
      //      cout << R0*R0.transpose() << endl << endl;
    }

    // set up rotation matrix for pos1
    void makeRot1()
    {
      Vector3 y;
      y << 0, 1, 0;
      R1.row(2) = normal1;
      y = y - normal1(1)*normal1;
      y.normalize();            // need to check if y is close to 0
      R1.row(1) = y;
      R1.row(0) = normal1.cross(R1.row(1));
    }

    // returns a precision matrix for point-plane
    Matrix3 prec0(number_t e)
    {
      makeRot0();
      Matrix3 prec;
      prec << e, 0, 0,
              0, e, 0,
              0, 0, 1;
      return R0.transpose()*prec*R0;
    }
    
    // returns a precision matrix for point-plane
    Matrix3 prec1(number_t e)
    {
      makeRot1();
      Matrix3 prec;
      prec << e, 0, 0,
              0, e, 0,
              0, 0, 1;
      return R1.transpose()*prec*R1;
    }
    
    // return a covariance matrix for plane-plane
    Matrix3 cov0(number_t e)
    {
      makeRot0();
      Matrix3 cov;
      cov  << 1, 0, 0,
              0, 1, 0,
              0, 0, e;
      return R0.transpose()*cov*R0;
    }
    
    // return a covariance matrix for plane-plane
    Matrix3 cov1(number_t e)
    {
      makeRot1();
      Matrix3 cov;
      cov  << 1, 0, 0,
              0, 1, 0,
              0, 0, e;
      return R1.transpose()*cov*R1;
    }

  };


  // 3D rigid constraint
  //    3 values for position wrt frame
  //    3 values for normal wrt frame, not used here
  // first two args are the measurement type, second two the connection classes
  class G2O_TYPES_ICP_API Edge_V_V_GICP : public  BaseBinaryEdge<3, EdgeGICP, VertexSE3, VertexSE3>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Edge_V_V_GICP() : pl_pl(false) {}
    Edge_V_V_GICP(const Edge_V_V_GICP* e);

    // switch to go between point-plane and plane-plane
    bool pl_pl;
    Matrix3 cov0, cov1;

    // I/O functions
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 3-vector
    void computeError()
    {
      // from <ViewPoint> to <Point>
      const VertexSE3 *vp0 = static_cast<const VertexSE3*>(_vertices[0]);
      const VertexSE3 *vp1 = static_cast<const VertexSE3*>(_vertices[1]);

      // get vp1 point into vp0 frame
      // could be more efficient if we computed this transform just once
      Vector3 p1;

#if 0
      if (_cnum >= 0 && 0)      // using global cache
        {
          if (_tainted[_cnum])  // set up transform
            {
              _transforms[_cnum] = vp0->estimate().inverse() * vp1->estimate();
              _tainted[_cnum] = 0;
              cout << _transforms[_cnum] << endl;
            }
          p1 = _transforms[_cnum].map(measurement().pos1); // do the transform
        }
      else
#endif
        {
          p1 = vp1->estimate() * measurement().pos1;
          p1 = vp0->estimate().inverse() * p1;
        }

      //      cout << endl << "Error computation; points are: " << endl;
      //      cout << p0.transpose() << endl;
      //      cout << p1.transpose() << endl;

      // get their difference
      // this is simple Euclidean distance, for now
      _error = p1 - measurement().pos0;

#if 0
      cout << "vp0" << endl << vp0->estimate() << endl;
      cout << "vp1" << endl << vp1->estimate() << endl;
      cout << "e Jac Xj" << endl <<  _jacobianOplusXj << endl << endl;
      cout << "e Jac Xi" << endl << _jacobianOplusXi << endl << endl;
#endif

      if (!pl_pl) return;

      // re-define the information matrix
      // topLeftCorner<3,3>() is the rotation()
      const Matrix3 transform = ( vp0->estimate().inverse() *  vp1->estimate() ).matrix().topLeftCorner<3,3>();
      information() = ( cov0 + transform * cov1 * transform.transpose() ).inverse();

    }

    // try analytic jacobians
#ifdef GICP_ANALYTIC_JACOBIANS
    virtual void linearizeOplus();
#endif

    // global derivative matrices
    static Matrix3 dRidx;
	static Matrix3 dRidy;
	static Matrix3 dRidz; // differential quat matrices
  };


/**
 * \brief Stereo camera vertex, derived from SE3 class.
 * Note that we use the actual pose of the vertex as its parameterization, rather
 * than the transform from RW to camera coords.
 * Uses static vars for camera params, so there is a single camera setup.
 */
  class G2O_TYPES_ICP_API VertexSCam : public VertexSE3
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexSCam();

      // I/O
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      // capture the update function to reset aux transforms
      virtual void oplusImpl(const number_t* update)
      {
        VertexSE3::oplusImpl(update);
        setAll();
      }

      // camera matrix and stereo baseline
      static Matrix3 Kcam;
      static number_t baseline;

      // transformations
      Eigen::Matrix<number_t,3,4,Eigen::ColMajor> w2n; // transform from world to node coordinates
      Eigen::Matrix<number_t,3,4,Eigen::ColMajor> w2i; // transform from world to image coordinates

      // Derivatives of the rotation matrix transpose wrt quaternion xyz, used for
      // calculating Jacobian wrt pose of a projection.
      Matrix3 dRdx, dRdy, dRdz;

      // transforms
      static void transformW2F(Eigen::Matrix<number_t,3,4,Eigen::ColMajor> &m,
                               const Vector3 &trans,
                               const Quaternion &qrot)
        {
          m.block<3,3>(0,0) = qrot.toRotationMatrix().transpose();
          m.col(3).setZero();         // make sure there's no translation
          Vector4 tt;
          tt.head(3) = trans;
          tt[3] = 1.0;
          m.col(3) = -m*tt;
        }

      static void transformF2W(Eigen::Matrix<number_t,3,4,Eigen::ColMajor> &m,
                               const Vector3 &trans,
                               const Quaternion &qrot)
        {
          m.block<3,3>(0,0) = qrot.toRotationMatrix();
          m.col(3) = trans;
        }

      // set up camera matrix
      static void setKcam(number_t fx, number_t fy, number_t cx, number_t cy, number_t tx)
      {
        Kcam.setZero();
        Kcam(0,0) = fx;
        Kcam(1,1) = fy;
        Kcam(0,2) = cx;
        Kcam(1,2) = cy;
        Kcam(2,2) = 1.0;
        baseline = tx;
      }

      // set transform from world to cam coords
      void setTransform()  {
        w2n = estimate().inverse().matrix().block<3,4>(0, 0);
        //transformW2F(w2n,estimate().translation(), estimate().rotation());
      }

      // Set up world-to-image projection matrix (w2i), assumes camera parameters
      // are filled.
      void setProjection() { w2i = Kcam * w2n; }

      // sets angle derivatives
      void setDr()
      {
        // inefficient, just for testing
        // use simple multiplications and additions for production code in calculating dRdx,y,z
        // for dS'*R', with dS the incremental change
        dRdx = dRidx * w2n.block<3,3>(0,0);
        dRdy = dRidy * w2n.block<3,3>(0,0);
        dRdz = dRidz * w2n.block<3,3>(0,0);
      }

      // set all aux transforms
      void setAll()
      {
        setTransform();
        setProjection();
        setDr();
      }

      // calculate stereo projection
      void mapPoint(Vector3 &res, const Vector3 &pt3)
      {
        Vector4 pt;
        pt.head<3>() = pt3;
        pt(3) = cst(1.0);
        Vector3 p1 = w2i * pt;
        Vector3 p2 = w2n * pt;
        Vector3 pb(baseline,0,0);

        number_t invp1 = cst(1.0)/p1(2);
        res.head<2>() = p1.head<2>()*invp1;

        // right camera px
        p2 = Kcam*(p2-pb);
        res(2) = p2(0)/p2(2);
      }

      static Matrix3 dRidx;
	  static Matrix3 dRidy;
	  static Matrix3 dRidz;
    };


/**
 * \brief Point vertex, XYZ, is in types_sba
 */


// stereo projection
// first two args are the measurement type, second two the connection classes
  class G2O_TYPES_ICP_API Edge_XYZ_VSC : public  BaseBinaryEdge<3, Vector3, VertexSBAPointXYZ, VertexSCam>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      Edge_XYZ_VSC();

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 2-vector
    void computeError()
    {
      // from <Point> to <Cam>
      const VertexSBAPointXYZ *point = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
      VertexSCam *cam = static_cast<VertexSCam*>(_vertices[1]);
      //cam->setAll();

      // calculate the projection
      Vector3 kp;
      cam->mapPoint(kp,point->estimate());

      // std::cout << std::endl << "CAM   " << cam->estimate() << std::endl;
      // std::cout << "POINT " << pt.transpose() << std::endl;
      // std::cout << "PROJ  " << p1.transpose() << std::endl;
      // std::cout << "PROJ  " << p2.transpose() << std::endl;
      // std::cout << "CPROJ " << kp.transpose() << std::endl;
      // std::cout << "MEAS  " << _measurement.transpose() << std::endl;

      // error, which is backwards from the normal observed - calculated
      // _measurement is the measured projection
      _error = kp - _measurement;
    }
#ifdef SCAM_ANALYTIC_JACOBIANS
    // jacobian
    virtual void linearizeOplus();
#endif
};



} // end namespace

#endif // TYPES_ICP
