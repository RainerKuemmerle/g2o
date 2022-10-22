// CSparse: a Concise Sparse matrix package.
// Copyright (c) 2005, Timothy A. Davis.
// http://www.cise.ufl.edu/research/sparse/CSparse
//
// --------------------------------------------------------------------------------
//
// CSparse is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 1.1 of the License, or (at your option) any later version.
//
// CSparse is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this Module; if not, write to the Free Software
// Foundation, Inc., 50 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "csparse_extension.h"

#include <cassert>
#include <iostream>

#include "g2o/stuff/macros.h"

namespace g2o {
namespace csparse_extension {

/**
 * Originally from CSparse, avoid memory re-allocations by giving workspace
 * pointers CSparse: Copyright (c) 2006-2011, Timothy A. Davis.
 */
int cs_cholsolsymb(const cs *A, number_t *b, const css *S, number_t *x,
                   int *work) {
  if (!CS_CSC(A) || !b || !S || !x) {
    std::cerr << __PRETTY_FUNCTION__ << ": No valid input!" << std::endl;
    assert(0);  // get a backtrace in debug mode
    return (0); /* check inputs */
  }
  const int n = A->n;
  csn* N = cs_chol_workspace(A, S, work, x); /* numeric Cholesky factorization */
  if (!N) {
    std::cerr << __PRETTY_FUNCTION__ << ": cholesky failed!" << std::endl;
    /*assert(0);*/
  }
  const int ok = static_cast<int>(N != nullptr);
  if (ok) {
    cs_ipvec(S->pinv, b, x, n); /* x = P*b */
    cs_lsolve(N->L, x);         /* x = L\x */
    cs_ltsolve(N->L, x);        /* x = L'\x */
    cs_pvec(S->pinv, x, b, n);  /* b = P'*x */
  }
  cs_nfree(N);
  return (ok);
}

/**
 * Originally from CSparse, avoid memory re-allocations by giving workspace
 * pointers CSparse: Copyright (c) 2006-2011, Timothy A. Davis.
 */
/* L = chol (A, [pinv parent cp]), pinv is optional */
csn *cs_chol_workspace(const cs *A, const css *S, int *cin, number_t *xin) {
  number_t d;
  number_t lki;
  number_t *Lx;
  number_t *x;
  number_t *Cx;
  int top;
  int i;
  int p;
  int k;
  int n;
  int *Li;
  int *Lp;
  int *cp;
  int *pinv;
  int *s;
  int *c;
  int *parent;
  int *Cp;
  int *Ci;
  cs *L;
  cs *C;
  cs *E;
  csn *N;
  if (!CS_CSC(A) || !S || !S->cp || !S->parent) return (nullptr);
  n = A->n;
  N = static_cast<csn *>(cs_calloc(1, sizeof(csn))); /* allocate result */
  c = cin;                                           /* get int workspace */
  x = xin; /* get number_t workspace */
  cp = S->cp;
  pinv = S->pinv;
  parent = S->parent;
  C = pinv ? cs_symperm(A, pinv, 1) : const_cast<cs *>(A);
  E = pinv ? C : nullptr; /* E is alias for A, or a copy E=A(p,p) */
  if (!N || !c || !x || !C) return (cs_ndone(N, E, nullptr, nullptr, 0));
  s = c + n;
  Cp = C->p;
  Ci = C->i;
  Cx = C->x;
  N->L = L = cs_spalloc(n, n, cp[n], 1, 0); /* allocate result */
  if (!L) return (cs_ndone(N, E, nullptr, nullptr, 0));
  Lp = L->p;
  Li = L->i;
  Lx = L->x;
  for (k = 0; k < n; k++) Lp[k] = c[k] = cp[k];
  for (k = 0; k < n; k++) /* compute L(k,:) for L*L' = C */
  {
    /* --- Nonzero pattern of L(k,:) ------------------------------------ */
    top = cs_ereach(C, k, parent, s, c); /* find pattern of L(k,:) */
    x[k] = 0;                            /* x (0:k) is now zero */
    for (p = Cp[k]; p < Cp[k + 1]; p++)  /* x = full(triu(C(:,k))) */
    {
      if (Ci[p] <= k) x[Ci[p]] = Cx[p];
    }
    d = x[k]; /* d = C(k,k) */
    x[k] = 0; /* clear x for k+1st iteration */
    /* --- Triangular solve --------------------------------------------- */
    for (; top < n; top++) /* solve L(0:k-1,0:k-1) * x = C(:,k) */
    {
      i = s[top];             /* s [top..n-1] is pattern of L(k,:) */
      lki = x[i] / Lx[Lp[i]]; /* L(k,i) = x (i) / L(i,i) */
      x[i] = 0;               /* clear x for k+1st iteration */
      for (p = Lp[i] + 1; p < c[i]; p++) {
        x[Li[p]] -= Lx[p] * lki;
      }
      d -= lki * lki; /* d = d - L(k,i)*L(k,i) */
      p = c[i]++;
      Li[p] = k; /* store L(k,i) in column i */
      Lx[p] = lki;
    }
    /* --- Compute L(k,k) ----------------------------------------------- */
    if (d <= 0) return (cs_ndone(N, E, nullptr, nullptr, 0)); /* not pos def */
    p = c[k]++;
    Li[p] = k; /* store L(k,k) = sqrt (d) in column k */
    Lx[p] = sqrt(d);
  }
  Lp[n] = cp[n]; /* finalize L */
  return (
      cs_ndone(N, E, nullptr, nullptr, 1)); /* success: free E,s,x; return N */
}

}  // namespace csparse_extension
}  // namespace g2o
