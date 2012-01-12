// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
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

#ifndef G2O_SIM_3
#define G2O_SIM_3

#include "g2o/types/slam3d/se3_ops.h"
#include <Eigen/Geometry>

namespace g2o
{
  using namespace Eigen;

  typedef  Matrix <double, 7, 1> Vector7d;
  typedef  Matrix <double, 7, 7> Matrix7d;
  

  struct Sim3
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:
    Quaterniond r;
    Vector3d t;
    double s;


public:
    Sim3()
    {
      r.setIdentity();
      t.fill(0.);
      s=1.;
    }

    Sim3(const Quaterniond & r, const Vector3d & t, double s)
      : r(r),t(t),s(s)
    {
    }

    Sim3(const Matrix3d & R, const Vector3d & t, double s)
      : r(Quaterniond(R)),t(t),s(s)
    {
    }


    Sim3(const Vector7d & update)
    {

      Vector3d omega;
      for (int i=0; i<3; i++)
        omega[i]=update[i];

      Vector3d upsilon;
      for (int i=0; i<3; i++)
        upsilon[i]=update[i+3];

      double sigma = update[6];
      double theta = omega.norm();
      Matrix3d Omega = skew(omega);
      s = std::exp(sigma);
      Matrix3d Omega2 = Omega*Omega;
      Matrix3d I;
      I.setIdentity();
      Matrix3d R;

      double eps = 0.00001;
      double A,B,C;
      if (fabs(sigma)<eps)
      {
        C = 1;
        if (theta<eps)
        {
          A = 1./2.;
          B = 1./6.;
          R = (I + Omega + Omega*Omega);
        }
        else
        {
          double theta2= theta*theta;
          A = (1-cos(theta))/(theta2);
          B = (theta-sin(theta))/(theta2*theta);
          R = I + sin(theta)/theta *Omega + (1-cos(theta))/(theta*theta)*Omega2;
        }
      }
      else
      {
        C=(s-1)/sigma;
        if (theta<eps)
        {
          double sigma2= sigma*sigma;
          A = ((sigma-1)*s+1)/sigma2;
          B= ((0.5*sigma2-sigma+1)*s)/(sigma2*sigma);
          R = (I + Omega + Omega2);
        }
        else
        {
          R = I + sin(theta)/theta *Omega + (1-cos(theta))/(theta*theta)*Omega2;



          double a=s*sin(theta);
          double b=s*cos(theta);
          double theta2= theta*theta;
          double sigma2= sigma*sigma;

          double c=theta2+sigma2;
          A = (a*sigma+ (1-b)*theta)/(theta*c);
          B = (C-((b-1)*sigma+a*theta)/(c))*1./(theta2);

        }
      }
      r = Quaterniond(R);



      Matrix3d W = A*Omega + B*Omega2 + C*I;
      t = W*upsilon;
    }

     Vector3d map (const Vector3d& xyz) const {
      return s*(r*xyz) + t;
    }

    Vector7d log() const
    {
      Vector7d res;
      double sigma = std::log(s);

      

   
      Vector3d omega;
      Vector3d upsilon;


      Matrix3d R = r.toRotationMatrix();
      double d =  0.5*(R(0,0)+R(1,1)+R(2,2)-1);

      Matrix3d Omega;

      double eps = 0.00001;
      Matrix3d I = Matrix3d::Identity();

      double A,B,C;
      if (fabs(sigma)<eps)
      {
        C = 1;
        if (d>1-eps)
        {
          omega=0.5*deltaR(R);
          Omega = skew(omega);
          A = 1./2.;
          B = 1./6.;
        }
        else
        {
          double theta = acos(d);
          double theta2 = theta*theta;
          omega = theta/(2*sqrt(1-d*d))*deltaR(R);
          Omega = skew(omega);
          A = (1-cos(theta))/(theta2);
          B = (theta-sin(theta))/(theta2*theta);
        }
      }
      else
      {
        C=(s-1)/sigma;
        if (d>1-eps)
        {

          double sigma2 = sigma*sigma;
          omega=0.5*deltaR(R);
          Omega = skew(omega);
          A = ((sigma-1)*s+1)/(sigma2);
          B = ((0.5*sigma2-sigma+1)*s)/(sigma2*sigma);
        }
        else
        {
          double theta = acos(d);
          omega = theta/(2*sqrt(1-d*d))*deltaR(R);
          Omega = skew(omega);
          double theta2 = theta*theta;
          double a=s*sin(theta);
          double b=s*cos(theta);
          double c=theta2 + sigma*sigma;
          A = (a*sigma+ (1-b)*theta)/(theta*c);
          B = (C-((b-1)*sigma+a*theta)/(c))*1./(theta2);
        }
      }

      Matrix3d W = A*Omega + B*Omega*Omega + C*I;

      upsilon = W.lu().solve(t);


      for (int i=0; i<3; i++)
        res[i] = omega[i];

       for (int i=0; i<3; i++)
        res[i+3] = upsilon[i];

      res[6] = sigma;

      return res;
      
    }


    Sim3 inverse() const
    {
      return Sim3(r.conjugate(), r.conjugate()*((-1./s)*t), 1./s);
    }
    

    double operator[](int i) const
    {
      assert(i<8);
      if (i<4){

        return r.coeffs()[i];
      }
      if (i<7){
        return t[i-4];
      }
      return s;
    }

    double& operator[](int i)
    {
      assert(i<8);
      if (i<4){

        return r.coeffs()[i];
      }
      if (i<7)
      {
        return t[i-4];
      }
      return s;
    }

    Sim3 operator *(const Sim3& other) const {
      Sim3 ret;
      ret.r = r*other.r;
      ret.t=s*(r*other.t)+t;
      ret.s=s*other.s;
      return ret;
    }

    Sim3& operator *=(const Sim3& other){
      Sim3 ret=(*this)*other;
      *this=ret;
      return *this;
    }

    inline const Vector3d& translation() const {return t;}

    inline Vector3d& translation() {return t;}

    inline const Quaterniond& rotation() const {return r;}

    inline Quaterniond& rotation() {return r;}

    inline const double& scale() const {return s;}

    inline double& scale() {return s;}

  };

  inline std::ostream& operator <<(std::ostream& out_str,
                                   const Sim3& sim3)
  {
    out_str << sim3.rotation().coeffs() << std::endl;
    out_str << sim3.translation() << std::endl;
    out_str << sim3.scale() << std::endl;

    return out_str;
  }

} // end namespace


#endif
