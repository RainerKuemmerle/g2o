// CSparse: a Concise Sparse matrix package.
// Copyright (c) 2005, Timothy A. Davis.
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
// Foundation, Inc., 50 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "csparse_extension.h"

#include <cassert>

#include "g2o/stuff/logger.h"

namespace g2o::csparse_extension {

/**
 * Originally from CSparse, avoid memory re-allocations by giving workspace
 * pointers CSparse: Copyright (c) 2006-2011, Timothy A. Davis.
 */
int cs_cholsolsymb(const cs* A, double* b, const css* S, double* x, int* work) {
  csn* N;
  int n, ok;
  if (!CS_CSC(A) || !b || !S || !x) {
    G2O_DEBUG("No valid input!");
    assert(0);  // get a backtrace in debug mode
    return (0); /* check inputs */
  }
  n = A->n;
  N = cs_chol_workspace(A, S, work, x); /* numeric Cholesky factorization */
  if (!N) {
    G2O_DEBUG("cholesky failed!");
  }
  ok = (N != NULL);
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
csn* cs_chol_workspace(const cs* A, const css* S, int* cin, double* xin) {
  double lki, *Lx, *x, *Cx;
  int i, p, k, n, *Li, *Lp, *cp, *pinv, *s, *c, *parent, *Cp, *Ci;
  cs *L, *C, *E;
  csn* N;
  if (!CS_CSC(A) || !S || !S->cp || !S->parent) return (NULL);
  n = A->n;
  N = static_cast<csn*>(cs_calloc(1, sizeof(csn))); /* allocate result */
  c = cin;                                          /* get int workspace */
  x = xin;                                          /* get double workspace */
  cp = S->cp;
  pinv = S->pinv;
  parent = S->parent;
  C = pinv ? cs_symperm(A, pinv, 1) : const_cast<cs*>(A);
  E = pinv ? C : NULL; /* E is alias for A, or a copy E=A(p,p) */
  if (!N || !c || !x || !C) return (cs_ndone(N, E, NULL, NULL, 0));
  s = c + n;
  Cp = C->p;
  Ci = C->i;
  Cx = C->x;
  N->L = L = cs_spalloc(n, n, cp[n], 1, 0); /* allocate result */
  if (!L) return (cs_ndone(N, E, NULL, NULL, 0));
  Lp = L->p;
  Li = L->i;
  Lx = L->x;
  for (k = 0; k < n; k++) Lp[k] = c[k] = cp[k];
  for (k = 0; k < n; k++) /* compute L(k,:) for L*L' = C */
  {
    /* --- Nonzero pattern of L(k,:) ------------------------------------ */
    int top = cs_ereach(C, k, parent, s, c); /* find pattern of L(k,:) */
    x[k] = 0;                                /* x (0:k) is now zero */
    for (p = Cp[k]; p < Cp[k + 1]; p++)      /* x = full(triu(C(:,k))) */
    {
      if (Ci[p] <= k) x[Ci[p]] = Cx[p];
    }
    double d = x[k]; /* d = C(k,k) */
    x[k] = 0;        /* clear x for k+1st iteration */
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
    if (d <= 0) return (cs_ndone(N, E, NULL, NULL, 0)); /* not pos def */
    p = c[k]++;
    Li[p] = k; /* store L(k,k) = sqrt (d) in column k */
    Lx[p] = sqrt(d);
  }
  Lp[n] = cp[n];                          /* finalize L */
  return (cs_ndone(N, E, NULL, NULL, 1)); /* success: free E,s,x; return N */
}

}  // namespace g2o::csparse_extension
