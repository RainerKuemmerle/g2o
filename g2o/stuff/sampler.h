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

#ifndef G2O_GAUSSIAN_SAMPLER_
#define G2O_GAUSSIAN_SAMPLER_

#include <Eigen/Core>
#include <memory>
#include <random>

#include "g2o_stuff_api.h"

namespace g2o {

double G2O_STUFF_API sampleUniform(double min = 0, double max = 1,
                                   std::mt19937* generator = nullptr);
double G2O_STUFF_API sampleGaussian(std::mt19937* generator = nullptr);

template <class SampleType, class CovarianceType>
class GaussianSampler {
 public:
  GaussianSampler(GaussianSampler const&) = delete;
  GaussianSampler& operator=(const GaussianSampler&) = delete;
  explicit GaussianSampler(bool hasGenerator = true)
      : generator_(hasGenerator ? new std::mt19937 : nullptr) {}
  bool setDistribution(const CovarianceType& cov) {
    Eigen::LLT<CovarianceType> cholDecomp;
    cholDecomp.compute(cov);
    const bool status = cholDecomp.info() != Eigen::Success;
    if (!status) return false;
    cholesky_ = cholDecomp.matrixL();
    return true;
  }
  //! return a sample of the Gaussian distribution
  SampleType generateSample() {
    SampleType s;
    for (int i = 0; i < s.size(); i++) {
      s(i) = (generator_) ? sampleGaussian(generator_.get()) : sampleGaussian();
    }
    return cholesky_ * s;
  }
  //! seed the random number generator, returns false if not having an own
  //! generator.
  bool seed(unsigned int s) {
    if (!generator_) return false;
    generator_->seed(s);
    return true;
  }

 protected:
  CovarianceType cholesky_;
  std::unique_ptr<std::mt19937> generator_;
};

class G2O_STUFF_API Sampler {
 public:
  /**
   * Gaussian random with a mean and standard deviation. Uses the
   * Polar method of Marsaglia.
   */
  static double gaussRand(double mean, double sigma);

  /**
   * sample a number from a uniform distribution
   */
  static double uniformRand(double lowerBndr, double upperBndr);

  /**
   * default seed function using the current time in seconds
   */
  static void seedRand();

  /** seed the random number generator */
  static void seedRand(unsigned int seed);
};

}  // namespace g2o

#endif
