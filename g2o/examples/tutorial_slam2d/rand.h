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

#ifndef G2O_RAND_HH
#define G2O_RAND_HH

#include <cstdlib>
#include <cmath>
#include <ctime>

#include "g2o_tutorial_slam2d_api.h"

namespace g2o {

  namespace tutorial {

    /**
     * \brief generate random numbers
     */
    class G2O_TUTORIAL_SLAM2D_API Rand
    {
      public:
        /**
         * Gaussian random with a mean and standard deviation. Uses the
         * Polar method of Marsaglia.
         */
        static double gauss_rand(double mean, double sigma)
        {
          double x, y, r2;
          do {
            x = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
            y = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
            r2 = x * x + y * y;
          } while (r2 > 1.0 || r2 == 0.0);
          return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
        }

        /**
         * sample a number from a uniform distribution
         */
        static double uniform_rand(double lowerBndr, double upperBndr)
        {
          return lowerBndr + ((double) std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
        }

        /**
         * default seed function using the current time in seconds
         */
        static void seed_rand()
        {
          seed_rand(static_cast<unsigned int>(std::time(NULL)));
        }

        /** seed the random number generator */
        static void seed_rand(unsigned int seed)
        {
          std::srand(seed);
        }
    };

  } // end namespace
} // end namespace

#endif
