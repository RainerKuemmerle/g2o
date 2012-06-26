// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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

#ifndef G2O_ROBUST_KERNEL_H
#define G2O_ROBUST_KERNEL_H

#include <tr1/memory>
#include <Eigen/Core>

namespace g2o {

  /**
   * \brief base for all robust kernels
   */
  class RobustKernel
  {
    public:
      RobustKernel();
      explicit RobustKernel(double delta);
      virtual ~RobustKernel() {}
      /**
       * compute the scaling factor for a error:
       * The error is e^T Omega e
       * The output rho is
       * rho[0]: The actual scaled error value
       * rho[1]: First derivative of the scaling function
       * rho[2]: Second derivative of the scaling function
       */
      virtual void robustify(double squaredError, Eigen::Vector3d& rho) const = 0;

      /**
       * set the window size of the error. A squared error above delta^2 is considered
       * as outlier in the data.
       */
      virtual void setDelta(double delta);
      double delta() const { return _delta;}

    protected:
      double _delta;
  };
  typedef std::tr1::shared_ptr<RobustKernel> RobustKernelPtr;

  /**
   * \brief scale a robust kernel to another delta (window size)
   *
   * Scales a robust kernel to another window size. Useful, in case if
   * one implements a kernel which only is designed for a fixed window
   * size.
   */
  class RobustKernelScaleDelta : public RobustKernel
  {
    public:
      explicit RobustKernelScaleDelta(const RobustKernelPtr& kernel, double delta = 1.);
      explicit RobustKernelScaleDelta(double delta = 1.);

      const RobustKernelPtr kernel() const { return _kernel;}
      void setKernel(const RobustKernelPtr& ptr);

      void robustify(double error, Eigen::Vector3d& rho) const;

    protected:
      RobustKernelPtr _kernel;
  };

  /**
   * \brief Huber Loss Function
   *
   * Loss function as described by Huber
   * See http://en.wikipedia.org/wiki/Huber_loss_function
   *
   *               1/2    2
   * rho(x) = 2 d x    - d
   */
  class RobustKernelHuber : public RobustKernel
  {
    public:
      virtual void robustify(double e2, Eigen::Vector3d& rho) const;
  };

  /**
   * \brief Pseudo Huber Loss Function
   *
   * The smooth pseudo huber cost function:
   * See http://en.wikipedia.org/wiki/Huber_loss_function
   *
   *             2
   *    2       e
   * 2 d  (sqrt(-- + 1) - 1)
   *             2
   *            d
   */
  class RobustKernelPseudoHuber : public RobustKernel
  {
    public:
      virtual void robustify(double e2, Eigen::Vector3d& rho) const;
  };

} // end namespace g2o

#endif
