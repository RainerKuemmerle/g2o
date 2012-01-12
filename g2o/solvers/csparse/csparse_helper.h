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

#ifndef G2O_CSPARSE_HELPER_H
#define G2O_CSPARSE_HELPER_H

#ifndef NCOMPLEX
#define NCOMPLEX
#endif
#include <cs.h>

namespace g2o {

/**
 * write the sparse matrix to a file loadable with ocatve
 */
bool writeCs2Octave(const char* filename, const cs* A, bool upperTriangular = true);

// our extensions to csparse
csn* cs_chol_workspace (const cs *A, const css *S, int* cin, double* xin);
int cs_cholsolsymb(const cs *A, double *b, const css* S, double* workspace, int* work);

} // end namespace

#endif
