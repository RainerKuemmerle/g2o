// CSparse: a Concise Sparse matrix package.
// Copyright (c) 2006, Timothy A. Davis.
// http://www.cise.ufl.edu/research/sparse/CSparse
// 
// --------------------------------------------------------------------------------
// 
// CSparse is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// CSparse is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this Module; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#ifndef G2O_CSPARSE_EXTENSION_H
#define G2O_CSPARSE_EXTENSION_H

#ifndef NCOMPLEX
#define NCOMPLEX
#endif
#include <cs.h>

#include "g2o_csparse_extension_api.h"

namespace g2o {

  namespace csparse_extension {

// our extensions to csparse
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
  /**
   * Originally from CSparse, avoid memory re-allocations by giving workspace pointers
   * CSparse: Copyright (c) 2006-2011, Timothy A. Davis.
   */
G2O_CSPARSE_EXTENSION_API csn* cs_chol_workspace (const cs *A, const css *S, int* cin, number_t* xin);
G2O_CSPARSE_EXTENSION_API int cs_cholsolsymb(const cs *A, number_t *b, const css* S, number_t* workspace, int* work);

} // end namespace
} // end namespace

#endif
