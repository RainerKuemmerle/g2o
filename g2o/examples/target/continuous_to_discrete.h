#ifndef G2O_CONTINUOUS_TO_DISCRETE_H_
#define G2O_CONTINUOUS_TO_DISCRETE_H_

#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

// Form for fixed-size matrices
template <typename MatrixType>
void continuousToDiscrete(MatrixType& Fd, MatrixType& Qd, const MatrixType& Fc,
                          const MatrixType& Qc, double dt) {
  enum {
    kNx = MatrixType::ColsAtCompileTime,
    kNy = MatrixType::RowsAtCompileTime,
    kNX2 = 2 * MatrixType::RowsAtCompileTime
  };

  using DoubleSizedMatrixType =
      Eigen::Matrix<typename MatrixType::Scalar, kNX2, kNX2>;
  DoubleSizedMatrixType bigA(kNX2, kNX2);
  DoubleSizedMatrixType bigB(kNX2, kNX2);

  // Construct the "big A matrix"
  bigA.template topLeftCorner<kNx, kNx>() = -Fc * dt;
  bigA.template topRightCorner<kNx, kNx>() = Qc * dt;
  bigA.template bottomLeftCorner<kNx, kNx>().setZero();
  bigA.template bottomRightCorner<kNx, kNx>() = Fc.transpose() * dt;

  // bigB = expm(bigA)
  // Eigen::MatrixExponential<DoubleSizedMatrixType> me(bigA);
  // me.compute(bigB);
  bigB = bigA.exp();

  // Extract the discrete time components
  Fd = bigB.template bottomRightCorner<kNx, kNx>().transpose();
  Qd = Fd * bigB.template topRightCorner<kNx, kNx>();
}

#endif  // __CONTINUOUS_TO_DISCRETE_H__
