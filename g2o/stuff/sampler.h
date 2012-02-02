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

#ifdef _MSC_VER
#include <random>
#else
#include <tr1/random>
#endif

#include "g2o_stuff_api.h"

namespace g2o {

  double G2O_STUFF_API sampleUniform(double min=0, double max=1, std::tr1::ranlux_base_01* generator=0);
  double G2O_STUFF_API sampleGaussian(std::tr1::ranlux_base_01* generator = 0);

  template <class SampleType, class CovarianceType>
  class GaussianSampler {
  public:
    GaussianSampler(bool hasGenerator=true){
      _generator = 0;
      if (hasGenerator){
        _generator = new std::tr1::ranlux_base_01;
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
      _L=cholDecomp.matrixL();
    }
    SampleType generateSample() {
      SampleType s;
      for (int i=0; i<s.size(); i++){
        s(i) = (_generator) ? sampleGaussian(_generator) : sampleGaussian();
      }
      return _L*s;
    }
  protected:
    CovarianceType _L;
    std::tr1::ranlux_base_01* _generator;
  };

}

#endif
