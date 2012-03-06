#ifndef G2O_CONTINUOUS_TO_DISCRETE_H_
#define G2O_CONTINUOUS_TO_DISCRETE_H_

#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>

// Form for fixed-size matrices
template<typename MatrixType>
void continuousToDiscrete(MatrixType& Fd, MatrixType& Qd,
                          const MatrixType& Fc, const MatrixType& Qc, double dt)
{
  enum
  {
    NX = MatrixType::ColsAtCompileTime,
    NY = MatrixType::RowsAtCompileTime,
    NX2 = 2 * MatrixType::RowsAtCompileTime
  };

  typedef Eigen::Matrix<typename MatrixType::Scalar,NX2,NX2> DoubleSizedMatrixType;
  DoubleSizedMatrixType bigA(NX2,NX2), bigB(NX2,NX2);

  // Construct the "big A matrix"
  bigA.template topLeftCorner<NX,NX>()=-Fc*dt;
  bigA.template topRightCorner<NX,NX>()= Qc * dt;
  bigA.template bottomLeftCorner<NX,NX>().setZero();
  bigA.template bottomRightCorner<NX,NX>()=Fc.transpose() * dt;

  // bigB = expm(bigA)
  Eigen::MatrixExponential<DoubleSizedMatrixType> me(bigA);
  me.compute(bigB);

  // Extract the discrete time components
  Fd = bigB.template bottomRightCorner<NX,NX>().transpose();
  Qd = Fd * bigB.template topRightCorner<NX,NX>();
}

#endif // __CONTINUOUS_TO_DISCRETE_H__
