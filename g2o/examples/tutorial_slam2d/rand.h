// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef G2O_RAND_HH
#define G2O_RAND_HH

#include <cstdlib>
#include <cmath>
#include <ctime>

namespace g2o {

  namespace tutorial {

    /**
     * \brief generate random numbers
     */
    class Rand
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
