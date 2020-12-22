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
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <random>

#include "g2o_stuff_api.h"

namespace g2o {

number_t G2O_STUFF_API sampleUniform(number_t min = 0, number_t max = 1, std::mt19937* generator = 0);
number_t G2O_STUFF_API sampleGaussian(std::mt19937* generator = 0);

template <class SampleType, class CovarianceType>
class GaussianSampler {
 public:
  GaussianSampler(GaussianSampler const&) = delete;
  GaussianSampler& operator=(const GaussianSampler&) = delete;
  GaussianSampler(bool hasGenerator = true) : _generator(hasGenerator ? new std::mt19937 : nullptr) {}
  void setDistribution(const CovarianceType& cov) {
    Eigen::LLT<CovarianceType> cholDecomp;
    cholDecomp.compute(cov);
    if (cholDecomp.info() == Eigen::NumericalIssue) {
      assert(false && "Cholesky decomposition on the covariance matrix failed");
      return;
    }
    _cholesky = cholDecomp.matrixL();
  }
  //! return a sample of the Gaussian distribution
  SampleType generateSample() {
    SampleType s;
    for (int i = 0; i < s.size(); i++) {
      s(i) = (_generator) ? sampleGaussian(_generator.get()) : sampleGaussian();
    }
    return _cholesky * s;
  }
  //! seed the random number generator, returns false if not having an own generator.
  bool seed(int s) {
    if (!_generator) return false;
    _generator->seed(s);
    return true;
  }

 protected:
  CovarianceType _cholesky;
  std::unique_ptr<std::mt19937> _generator;
};

class G2O_STUFF_API Sampler {
 public:
  /**
   * Gaussian random with a mean and standard deviation. Uses the
   * Polar method of Marsaglia.
   */
  static number_t gaussRand(number_t mean, number_t sigma) {
    number_t y, r2;
    do {
      number_t x = -1.0 + 2.0 * uniformRand(0.0, 1.0);
      y = -1.0 + 2.0 * uniformRand(0.0, 1.0);
      r2 = x * x + y * y;
    } while (r2 > 1.0 || r2 == 0.0);
    return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
  }

  /**
   * sample a number from a uniform distribution
   */
  static number_t uniformRand(number_t lowerBndr, number_t upperBndr) {
    return lowerBndr + ((number_t)std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
  }
  /**
   * default seed function using the current time in seconds
   */
  static void seedRand() { seedRand(static_cast<unsigned int>(std::time(NULL))); }

  /** seed the random number generator */
  static void seedRand(unsigned int seed) { std::srand(seed); }
};

}  // namespace g2o

#endif
