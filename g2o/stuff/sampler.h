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
