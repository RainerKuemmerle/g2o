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

#include "csparse_helper.h"

#include "g2o/stuff/macros.h"

#include <string>
#include <cassert>
#include <fstream>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

namespace g2o {
namespace csparse_extension {

  struct SparseMatrixEntry{
    SparseMatrixEntry(int r=-1, int c=-1, double x=0.) :
      _r(r), _c(c), _x(x)
    {
    }
    int _r,_c;
    double _x;
  };

  struct SparseMatrixEntryColSort
  {
    bool operator()(const SparseMatrixEntry& e1, const SparseMatrixEntry& e2) const
    {
      return e1._c < e2._c || (e1._c == e2._c && e1._r < e2._r);
    }
  };

  /**
   * Originally from CSparse, avoid memory re-allocations by giving workspace pointers
   * CSparse: Copyright (c) 2006-2011, Timothy A. Davis.
   */
  int cs_cholsolsymb(const cs *A, double *b, const css* S, double* x, int* work)
  {
    csn *N ;
    int n, ok ;
    if (!CS_CSC (A) || !b || ! S || !x) {
      fprintf(stderr, "%s: No valid input!\n", __PRETTY_FUNCTION__);
      assert(0); // get a backtrace in debug mode
      return (0) ;     /* check inputs */
    }
    n = A->n ;
    N = cs_chol_workspace (A, S, work, x) ;                    /* numeric Cholesky factorization */
    if (!N) {
      fprintf(stderr, "%s: cholesky failed!\n", __PRETTY_FUNCTION__);
      /*assert(0);*/
    }
    ok = (N != NULL) ;
    if (ok)
    {
      cs_ipvec (S->pinv, b, x, n) ;   /* x = P*b */
      cs_lsolve (N->L, x) ;           /* x = L\x */
      cs_ltsolve (N->L, x) ;          /* x = L'\x */
      cs_pvec (S->pinv, x, b, n) ;    /* b = P'*x */
    }
    cs_nfree (N) ;
    return (ok) ;
  }

  /**
   * Originally from CSparse, avoid memory re-allocations by giving workspace pointers
   * CSparse: Copyright (c) 2006-2011, Timothy A. Davis.
   */
  /* L = chol (A, [pinv parent cp]), pinv is optional */
  csn* cs_chol_workspace (const cs *A, const css *S, int* cin, double* xin)
  {
    double d, lki, *Lx, *x, *Cx ;
    int top, i, p, k, n, *Li, *Lp, *cp, *pinv, *s, *c, *parent, *Cp, *Ci ;
    cs *L, *C, *E ;
    csn *N ;
    if (!CS_CSC (A) || !S || !S->cp || !S->parent) return (NULL) ;
    n = A->n ;
    N = (csn*) cs_calloc (1, sizeof (csn)) ;       /* allocate result */
    c = cin ;     /* get int workspace */
    x = xin ;    /* get double workspace */
    cp = S->cp ; pinv = S->pinv ; parent = S->parent ;
    C = pinv ? cs_symperm (A, pinv, 1) : ((cs *) A) ;
    E = pinv ? C : NULL ;           /* E is alias for A, or a copy E=A(p,p) */
    if (!N || !c || !x || !C) return (cs_ndone (N, E, NULL, NULL, 0)) ;
    s = c + n ;
    Cp = C->p ; Ci = C->i ; Cx = C->x ;
    N->L = L = cs_spalloc (n, n, cp [n], 1, 0) ;    /* allocate result */
    if (!L) return (cs_ndone (N, E, NULL, NULL, 0)) ;
    Lp = L->p ; Li = L->i ; Lx = L->x ;
    for (k = 0 ; k < n ; k++) Lp [k] = c [k] = cp [k] ;
    for (k = 0 ; k < n ; k++)       /* compute L(k,:) for L*L' = C */
    {
      /* --- Nonzero pattern of L(k,:) ------------------------------------ */
      top = cs_ereach (C, k, parent, s, c) ;      /* find pattern of L(k,:) */
      x [k] = 0 ;                                 /* x (0:k) is now zero */
      for (p = Cp [k] ; p < Cp [k+1] ; p++)       /* x = full(triu(C(:,k))) */
      {
        if (Ci [p] <= k) x [Ci [p]] = Cx [p] ;
      }
      d = x [k] ;                     /* d = C(k,k) */
      x [k] = 0 ;                     /* clear x for k+1st iteration */
      /* --- Triangular solve --------------------------------------------- */
      for ( ; top < n ; top++)    /* solve L(0:k-1,0:k-1) * x = C(:,k) */
      {
        i = s [top] ;               /* s [top..n-1] is pattern of L(k,:) */
        lki = x [i] / Lx [Lp [i]] ; /* L(k,i) = x (i) / L(i,i) */
        x [i] = 0 ;                 /* clear x for k+1st iteration */
        for (p = Lp [i] + 1 ; p < c [i] ; p++)
        {
          x [Li [p]] -= Lx [p] * lki ;
        }
        d -= lki * lki ;            /* d = d - L(k,i)*L(k,i) */
        p = c [i]++ ;
        Li [p] = k ;                /* store L(k,i) in column i */
        Lx [p] = lki ;
      }
      /* --- Compute L(k,k) ----------------------------------------------- */
      if (d <= 0) return (cs_ndone (N, E, NULL, NULL, 0)) ; /* not pos def */
      p = c [k]++ ;
      Li [p] = k ;                /* store L(k,k) = sqrt (d) in column k */
      Lx [p] = sqrt (d) ;
    }
    Lp [n] = cp [n] ;               /* finalize L */
    return (cs_ndone (N, E, NULL, NULL, 1)) ; /* success: free E,s,x; return N */
  }

  bool writeCs2Octave(const char* filename, const cs* A, bool upperTriangular)
  {
    int cols = A->n;
    int rows = A->m;

    string name = filename;
    std::string::size_type lastDot = name.find_last_of('.');
    if (lastDot != std::string::npos) 
      name = name.substr(0, lastDot);

    vector<SparseMatrixEntry> entries;
    if (A->nz == -1) { // CCS matrix
      const int* Ap = A->p;
      const int* Ai = A->i;
      const double* Ax = A->x;
      for (int i=0; i < cols; i++) {
        const int& rbeg = Ap[i];
        const int& rend = Ap[i+1];
        for (int j = rbeg; j < rend; j++) {
          entries.push_back(SparseMatrixEntry(Ai[j], i, Ax[j]));
          if (upperTriangular && Ai[j] != i)
            entries.push_back(SparseMatrixEntry(i, Ai[j], Ax[j]));
        }
      }
    } else { // Triplet matrix
      entries.reserve(A->nz);
      int *Aj = A->p;             // column indeces
      int *Ai = A->i;             // row indices
      double *Ax = A->x;          // values;
      for (int i = 0; i < A->nz; ++i) {
        entries.push_back(SparseMatrixEntry(Ai[i], Aj[i], Ax[i]));
        if (upperTriangular && Ai[i] != Aj[i])
          entries.push_back(SparseMatrixEntry(Aj[i], Ai[i], Ax[i]));
      }
    }
    sort(entries.begin(), entries.end(), SparseMatrixEntryColSort());

    std::ofstream fout(filename);
    fout << "# name: " << name << std::endl;
    fout << "# type: sparse matrix" << std::endl;
    fout << "# nnz: " << entries.size() << std::endl;
    fout << "# rows: " << rows << std::endl;
    fout << "# columns: " << cols << std::endl;
    //fout << fixed;
    fout << setprecision(9) << endl;

    for (vector<SparseMatrixEntry>::const_iterator it = entries.begin(); it != entries.end(); ++it) {
      const SparseMatrixEntry& entry = *it;
      fout << entry._r+1 << " " << entry._c+1 << " " << entry._x << std::endl;
    }
    return fout.good();
  }

} // end namespace
} // end namespace
