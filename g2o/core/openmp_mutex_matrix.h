// g2o - General Graph Optimization
// Copyright (C) 2012 R. Kuemmerle, G. Grisetti, W. Burgard
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

#include "g2o_core_api.h"

#ifdef _MSC_VER
#include <unordered_map>
#else
#include <tr1/unordered_map>
#endif
#include <cassert>
#include <vector>

#include "openmp_mutex.h"

namespace g2o {

  /**
   * \brief A matrix of mutexes
   */
  class OpenMPMutexMatrix
  {
    public:
      typedef std::tr1::unordered_map<int, OpenMPMutex*>      SparseRow;
      typedef std::vector<SparseRow>                          ColumnStorage;

      OpenMPMutexMatrix();
      ~OpenMPMutexMatrix();

      /**
       * reserve memory for a certain number of columns
       */
      void resizeColumns(int cols);

      /**
       * return a mutex, if alloc is true the mutex will be allocated
       * if it does not exist.
       */
      OpenMPMutex* mutex(int r, int c, bool allocate = false);

      void deallocate();

    protected:
      ColumnStorage _storage;

    private:
      // Disable the copy constructor and assignment operator
      OpenMPMutexMatrix(const OpenMPMutexMatrix&);
      OpenMPMutexMatrix& operator= (const OpenMPMutexMatrix&);
  };

} // end namespace g2o
