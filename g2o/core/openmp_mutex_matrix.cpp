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

#include "openmp_mutex_matrix.h"

using namespace std;

namespace g2o {

OpenMPMutexMatrix::OpenMPMutexMatrix()
{
}

OpenMPMutexMatrix::~OpenMPMutexMatrix()
{
  deallocate();
}

OpenMPMutex* OpenMPMutexMatrix::mutex(int r, int c, bool allocate)
{
  SparseRow::iterator foundIt = _storage[c].find(r);
  if (foundIt == _storage[c].end()) {
    if (! allocate) {
      return 0;
    }
    OpenMPMutex* m = new OpenMPMutex;
    _storage[c].insert(std::make_pair(r, m));
    return m;
  }
  return foundIt->second;
}

void OpenMPMutexMatrix::deallocate()
{
  for (size_t i=0; i < _storage.size(); ++i) {
    SparseRow& row = _storage[i];
    for (SparseRow::iterator it = row.begin(); it != row.end(); ++it)
      delete it->second;
    row.clear();
  }
  _storage.clear();
}

void OpenMPMutexMatrix::resizeColumns(int cols)
{
  _storage.resize(cols);
}

} // end namespace
