#include "sampler.h"

namespace g2o {

  static std::tr1::normal_distribution<double> _univariateSampler(0., 1.);
  static std::tr1::uniform_real<double> _uniformReal;
  static std::tr1::ranlux_base_01* _gen_real = new std::tr1::ranlux_base_01;
 
  double sampleUniform(double min, double max, std::tr1::ranlux_base_01* generator){
    if (generator)
      return _uniformReal(*generator)*(max-min)+min;
    return _uniformReal(*_gen_real)*(max-min)+min;
  }
  
  double sampleGaussian(std::tr1::ranlux_base_01* generator){
    if (generator)
      return _univariateSampler(*generator);
    return _univariateSampler(*_gen_real);
  }

}
