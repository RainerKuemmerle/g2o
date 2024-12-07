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

#include "sampler.h"

#include <cmath>
#include <cstdlib>
#include <ctime>

namespace g2o {

namespace {
std::mt19937 _gen_real;
}  // namespace

double sampleUniform(double min, double max, std::mt19937* generator) {
  static std::uniform_real_distribution<double> uniform_real;
  std::uniform_real_distribution<double>::param_type params(min, max);
  if (generator) return uniform_real(*generator, params);
  return uniform_real(_gen_real, params);
}

double sampleGaussian(std::mt19937* generator) {
  static std::normal_distribution<double> _univariateSampler(0., 1.);
  if (generator) return _univariateSampler(*generator);
  return _univariateSampler(_gen_real);
}

double Sampler::gaussRand(double mean, double sigma) {
  double y;
  double r2;
  do {
    double x = -1.0 + (2.0 * uniformRand(0.0, 1.0));
    y = -1.0 + 2.0 * uniformRand(0.0, 1.0);
    r2 = x * x + y * y;
  } while (r2 > 1.0 || r2 == 0.0);
  return mean + (sigma * y * std::sqrt(-2.0 * log(r2) / r2));
}

double Sampler::uniformRand(double lowerBndr, double upperBndr) {
  return lowerBndr + ((static_cast<double>(std::rand()) / (RAND_MAX + 1.0)) *
                      (upperBndr - lowerBndr));
}

void Sampler::seedRand() {
  seedRand(static_cast<unsigned int>(std::time(nullptr)));
}

void Sampler::seedRand(unsigned int seed) { std::srand(seed); }

}  // namespace g2o
