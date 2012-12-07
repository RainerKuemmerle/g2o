#ifndef G2O_PLANE3D_H_
#define G2O_PLANE3D_H_

#include "g2o/stuff/misc.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace g2o {
  
  class Plane3D {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      friend Plane3D operator*(const Eigen::Isometry3d& t, const Plane3D& plane);

      Plane3D(){
        Vector4d v;
        v << 1., 0., 0., -1.;
        fromVector(v);
      }

      Plane3D(const Vector4d& v){
        fromVector(v);
      }


      inline Vector4d toVector() const {
        return _coeffs;
      }

      inline void fromVector(const Vector4d& coeffs_) {
        _coeffs=coeffs_;
        normalize(_coeffs);
      }

      static double azimuth(const Vector3d& v) {
        return atan2(v(1),v(0));
      }

      static  double elevation(const Vector3d& v) {
        return atan2(v(2), v.head<2>().norm());
      }

    double distance() const {
      return -_coeffs(3);
    }

    Vector3d normal() const {
      return _coeffs.head<3>();
    }

    
    static Matrix3d rotation(const Vector3d& v)  {
      double _azimuth = azimuth(v);
      double _elevation = elevation(v); 
      return (AngleAxisd(_azimuth,  Vector3d::UnitZ())*AngleAxisd(- _elevation, Vector3d::UnitY())).toRotationMatrix();
    }

    void oplus(Vector3d v){
      //construct a normal from azimuth and evelation;
      double _azimuth=v[0];
      double _elevation=v[1];
      double s=sin(_elevation), c=cos(_elevation);
      Vector3d n (c*cos(_azimuth), c*sin(_azimuth), s) ;
      
      // rotate the normal
      Matrix3d R=rotation(normal());
      double d=distance()*exp(v[2]);
      _coeffs.head<3>() = R*n;
      _coeffs(3) = -d;
      normalize(_coeffs);
    }
    
    Vector3d ominus(const Plane3D& plane){
      //construct the rotation that would bring the plane normal in (1 0 0)
      Matrix3d R=rotation(plane.normal()).transpose();
      Vector3d n=R*normal();
      double logD1=log(distance());
      double logD2=log(plane.distance());
      return Vector3d(azimuth(n), elevation(n), logD1-logD2);
    }

    protected:

      static inline void normalize(Vector4d& coeffs) {
        double n=coeffs.head<3>().norm();
        coeffs = coeffs * (1./n);
        if (coeffs(3) > 0)
          coeffs = -coeffs;
      }

      Eigen::Vector4d _coeffs;
  };

  inline Plane3D operator*(const Isometry3d& t, const Plane3D& plane){
    Vector4d v=plane._coeffs;
    Vector4d v2;
    Matrix3d R=t.rotation();
    v2.head<3>() = R*v.head<3>();
    v2(3)=v(3) + t.translation().dot(v2.head<3>());
    return Plane3D(v2);
  };


}

#endif
