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

#ifndef G2O_OPENMP_MUTEX
#define G2O_OPENMP_MUTEX

#include "g2o/config.h"

#ifdef G2O_OPENMP
#include <omp.h>
#endif

namespace g2o {

#ifdef G2O_OPENMP

  /**
   * \brief Mutex realized via OpenMP
   */
  class OpenMPMutex
  {
    public:
      OpenMPMutex() { omp_init_lock(&_lock); }
      ~OpenMPMutex() { omp_destroy_lock(&_lock); }
      void lock() { omp_set_lock(&_lock); }
      void unlock() { omp_unset_lock(&_lock); }

    protected:
      omp_lock_t _lock;
  };

#else

  /*
   * dummy which does nothing in case we don't have OpenMP support
   */
  struct OpenMPMutex 
  {
    void lock() {}
    void unlock() {}
  };

#endif

}

#endif
