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

#ifndef G2O_TYPES_ICP
#define G2O_TYPES_ICP

#define GICP_ANALYTIC_JACOBIANS
//#define SCAM_ANALYTIC_JACOBIANS

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/types/sba/types_sba.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include <Eigen/Geometry>
#include <iostream>

namespace g2o {

  using namespace Eigen;
  using namespace std;
  typedef  Matrix<double, 6, 1> Vector6d;



//
// GICP-type edges
// Each measurement is between two rigid points on each 6DOF vertex
//


  //
  // class for edges between two points rigidly attached to vertices
  //

  class EdgeGICP
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   public:
    // point positions
    Vector3d pos0, pos1;

    // unit normals
    Vector3d normal0, normal1;

    // rotation matrix for normal
    Matrix3d R0,R1;

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
      Vector3d y;
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
      Vector3d y;
      y << 0, 1, 0;
      R1.row(2) = normal1;
      y = y - normal1(1)*normal1;
      y.normalize();            // need to check if y is close to 0
      R1.row(1) = y;
      R1.row(0) = normal1.cross(R1.row(1));
    }

    // returns a precision matrix for point-plane
    Matrix3d prec0(double e)
    {
      makeRot0();
      Matrix3d prec;
      prec << e, 0, 0,
              0, e, 0,
              0, 0, 1;
      return R0.transpose()*prec*R0;
    }
    
    // returns a precision matrix for point-plane
    Matrix3d prec1(double e)
    {
      makeRot1();
      Matrix3d prec;
      prec << e, 0, 0,
              0, e, 0,
              0, 0, 1;
      return R1.transpose()*prec*R1;
    }
    
    // return a covariance matrix for plane-plane
    Matrix3d cov0(double e)
    {
      makeRot0();
      Matrix3d cov;
      cov  << 1, 0, 0,
              0, 1, 0,
              0, 0, e;
      return R0.transpose()*cov*R0;
    }
    
    // return a covariance matrix for plane-plane
    Matrix3d cov1(double e)
    {
      makeRot1();
      Matrix3d cov;
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
  class Edge_V_V_GICP : public  BaseBinaryEdge<3, EdgeGICP, VertexSE3, VertexSE3>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      Edge_V_V_GICP() : pl_pl(false) {};
      Edge_V_V_GICP(const Edge_V_V_GICP* e);

    // switch to go between point-plane and plane-plane
    bool pl_pl;
    Matrix3d cov0, cov1;

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
      Vector3d p1;

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
          p1 = vp1->estimate().map(measurement().pos1);
          p1 = vp0->estimate().inverse().map(p1);
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
      const Matrix3d transform = ( vp0->estimate().inverse() *  vp1->estimate() ).rotation().toRotationMatrix();
      information() = ( cov0 + transform * cov1 * transform.transpose() ).inverse();

    }

    // try analytic jacobians
#ifdef GICP_ANALYTIC_JACOBIANS
    virtual void linearizeOplus();
#endif

    // global derivative matrices
    static Matrix3d dRidx, dRidy, dRidz; // differential quat matrices
  };



/**
 * \brief Stereo camera vertex, derived from SE3 class.
 * Note that we use the actual pose of the vertex as its parameterization, rather
 * than the transform from RW to camera coords.
 * Uses static vars for camera params, so there is a single camera setup.
 */


  class VertexSCam : public VertexSE3
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexSCam();

      // I/O
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      // capture the update function to reset aux transforms
      virtual void oplusImpl(const double* update)
      {
        VertexSE3::oplus(update);
        setAll();
      }

      // camera matrix and stereo baseline
      static Matrix3d Kcam;
      static double baseline;

      // transformations
      Matrix<double,3,4> w2n; // transform from world to node coordinates
      Matrix<double,3,4> w2i; // transform from world to image coordinates

      // Derivatives of the rotation matrix transpose wrt quaternion xyz, used for
      // calculating Jacobian wrt pose of a projection.
      Matrix3d dRdx, dRdy, dRdz;

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
      static void setKcam(double fx, double fy, double cx, double cy, double tx)
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
      void setTransform()  { transformW2F(w2n,estimate().translation(),
                                          estimate().rotation()); }

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
      void mapPoint(Vector3d &res, const Vector3d &pt3)
      {
        Vector4d pt;
        pt.head<3>() = pt3;
        pt(3) = 1.0;
        Vector3d p1 = w2i * pt;
        Vector3d p2 = w2n * pt;
        Vector3d pb(baseline,0,0);

        double invp1 = 1.0/p1(2);
        res.head<2>() = p1.head<2>()*invp1;

        // right camera px
        p2 = Kcam*(p2-pb);
        res(2) = p2(0)/p2(2);
      }

      static Matrix3d dRidx, dRidy, dRidz;
    };


/**
 * \brief Point vertex, XYZ, is in types_sba
 */


// stereo projection
// first two args are the measurement type, second two the connection classes
  class Edge_XYZ_VSC : public  BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSCam>
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
      Vector3d kp;
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
