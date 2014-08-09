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
#include <Eigen/Cholesky>
#include <Eigen/StdVector>

#include <cstdlib>
#include <cmath>
#include <ctime>

#include <random>

#include "g2o_stuff_api.h"

namespace g2o {

  double G2O_STUFF_API sampleUniform(double min=0, double max=1, std::mt19937* generator=0);
  double G2O_STUFF_API sampleGaussian(std::mt19937* generator = 0);

  template <class SampleType, class CovarianceType>
  class GaussianSampler {
  public:
    GaussianSampler(bool hasGenerator=true){
      _generator = 0;
      if (hasGenerator){
        _generator = new std::mt19937;
      }
    }
    ~GaussianSampler() {
      if (_generator)
        delete _generator;
    }
    void setDistribution(const CovarianceType& cov){
      Eigen::LLT<CovarianceType> cholDecomp;
      cholDecomp.compute(cov);
      if (cholDecomp.info()==Eigen::NumericalIssue)
        return;
      _cholesky=cholDecomp.matrixL();
    }
    SampleType generateSample() {
      SampleType s;
      for (int i=0; i<s.size(); i++){
        s(i) = (_generator) ? sampleGaussian(_generator) : sampleGaussian();
      }
      return _cholesky*s;
    }
  protected:
    CovarianceType _cholesky;
    std::mt19937* _generator;
  };

  class G2O_STUFF_API Sampler
  {
    public:
      /**
       * Gaussian random with a mean and standard deviation. Uses the
       * Polar method of Marsaglia.
       */
      static double gaussRand(double mean, double sigma)
      {
        double x, y, r2;
        do {
          x = -1.0 + 2.0 * uniformRand(0.0, 1.0);
          y = -1.0 + 2.0 * uniformRand(0.0, 1.0);
          r2 = x * x + y * y;
        } while (r2 > 1.0 || r2 == 0.0);
        return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
      }

      /**
       * sample a number from a uniform distribution
       */
      static double uniformRand(double lowerBndr, double upperBndr)
      {
        return lowerBndr + ((double) std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
      }
      /**
       * default seed function using the current time in seconds
       */
      static void seedRand()
      {
        seedRand(static_cast<unsigned int>(std::time(NULL)));
      }

      /** seed the random number generator */
      static void seedRand(unsigned int seed)
      {
        std::srand(seed);
      }
  };

}

#endif
