#ifndef _CS_H
#define _CS_H
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stddef.h>
#ifdef MATLAB_MEX_FILE
#include "mex.h"
#endif
#define CS_VER 3                    /* CSparse Version */
#define CS_SUBVER 1
#define CS_SUBSUB 0
#define CS_DATE "Jun 1, 2012"       /* CSparse release date */
#define CS_COPYRIGHT "Copyright (c) Timothy A. Davis, 2006-2012"

#include "cs_api.h"

#ifdef __cplusplus
extern "C" {
#endif

// rk: We define csi to be int to be backward compatible with older CSparse releases.
//     At some point we might adapt the internal structures of g2o to use the same type
//     as the one given is this header file.
#define csi int

/* -------------------------------------------------------------------------- */
/* In version 3.0.0 of CSparse, "int" is no longer used.  32-bit MATLAB is
   becoming more rare, as are 32-bit computers.  CSparse now uses "csi" as its
   basic integer, which is ptrdiff_t by default (the same as mwSignedIndex in a
   MATLAB mexFunction).  That makes the basic integer 32-bit on 32-bit
   computers and 64-bit on 64-bit computers.  It is #define'd below, in case
   you wish to change it to something else (back to "int" for example).  You
   can also compile with -Dcsi=int (or whatever) to change csi without editting
   this file. */
#ifdef MATLAB_MEX_FILE
#undef csi
#define csi mwSignedIndex
#endif
#ifndef csi
#define csi ptrdiff_t
#endif
/* -------------------------------------------------------------------------- */

/* --- primary CSparse routines and data structures ------------------------- */
typedef struct cs_sparse    /* matrix in compressed-column or triplet form */
{
    csi nzmax ;     /* maximum number of entries */
    csi m ;         /* number of rows */
    csi n ;         /* number of columns */
    csi *p ;        /* column pointers (size n+1) or col indices (size nzmax) */
    csi *i ;        /* row indices, size nzmax */
    double *x ;     /* numerical values, size nzmax */
    csi nz ;        /* # of entries in triplet matrix, -1 for compressed-col */
} cs ;

G2O_CSPARSE_API cs *cs_add (const cs *A, const cs *B, double alpha, double beta) ;
G2O_CSPARSE_API csi cs_cholsol (csi order, const cs *A, double *b) ;
G2O_CSPARSE_API cs *cs_compress (const cs *T) ;
G2O_CSPARSE_API csi cs_dupl (cs *A) ;
G2O_CSPARSE_API csi cs_entry (cs *T, csi i, csi j, double x) ;
G2O_CSPARSE_API csi cs_gaxpy (const cs *A, const double *x, double *y) ;
G2O_CSPARSE_API cs *cs_load (FILE *f) ;
G2O_CSPARSE_API csi cs_lusol (csi order, const cs *A, double *b, double tol) ;
G2O_CSPARSE_API cs *cs_multiply (const cs *A, const cs *B) ;
G2O_CSPARSE_API double cs_norm (const cs *A) ;
G2O_CSPARSE_API csi cs_print (const cs *A, csi brief) ;
G2O_CSPARSE_API csi cs_qrsol (csi order, const cs *A, double *b) ;
G2O_CSPARSE_API cs *cs_transpose (const cs *A, csi values) ;
/* utilities */
G2O_CSPARSE_API void *cs_calloc (csi n, size_t size) ;
G2O_CSPARSE_API void *cs_free (void *p) ;
G2O_CSPARSE_API void *cs_realloc (void *p, csi n, size_t size, csi *ok) ;
G2O_CSPARSE_API cs *cs_spalloc (csi m, csi n, csi nzmax, csi values, csi triplet) ;
G2O_CSPARSE_API cs *cs_spfree (cs *A) ;
G2O_CSPARSE_API csi cs_sprealloc (cs *A, csi nzmax) ;
G2O_CSPARSE_API void *cs_malloc (csi n, size_t size) ;

/* --- secondary CSparse routines and data structures ----------------------- */
typedef struct cs_symbolic  /* symbolic Cholesky, LU, or QR analysis */
{
    csi *pinv ;     /* inverse row perm. for QR, fill red. perm for Chol */
    csi *q ;        /* fill-reducing column permutation for LU and QR */
    csi *parent ;   /* elimination tree for Cholesky and QR */
    csi *cp ;       /* column pointers for Cholesky, row counts for QR */
    csi *leftmost ; /* leftmost[i] = min(find(A(i,:))), for QR */
    csi m2 ;        /* # of rows for QR, after adding fictitious rows */
    double lnz ;    /* # entries in L for LU or Cholesky; in V for QR */
    double unz ;    /* # entries in U for LU; in R for QR */
} css ;

typedef struct cs_numeric   /* numeric Cholesky, LU, or QR factorization */
{
    cs *L ;         /* L for LU and Cholesky, V for QR */
    cs *U ;         /* U for LU, R for QR, not used for Cholesky */
    csi *pinv ;     /* partial pivoting for LU */
    double *B ;     /* beta [0..n-1] for QR */
} csn ;

typedef struct cs_dmperm_results    /* cs_dmperm or cs_scc output */
{
    csi *p ;        /* size m, row permutation */
    csi *q ;        /* size n, column permutation */
    csi *r ;        /* size nb+1, block k is rows r[k] to r[k+1]-1 in A(p,q) */
    csi *s ;        /* size nb+1, block k is cols s[k] to s[k+1]-1 in A(p,q) */
    csi nb ;        /* # of blocks in fine dmperm decomposition */
    csi rr [5] ;    /* coarse row decomposition */
    csi cc [5] ;    /* coarse column decomposition */
} csd ;

G2O_CSPARSE_API csi *cs_amd (csi order, const cs *A) ;
G2O_CSPARSE_API csn *cs_chol (const cs *A, const css *S) ;
G2O_CSPARSE_API csd *cs_dmperm (const cs *A, csi seed) ;
G2O_CSPARSE_API csi cs_droptol (cs *A, double tol) ;
G2O_CSPARSE_API csi cs_dropzeros (cs *A) ;
G2O_CSPARSE_API csi cs_happly (const cs *V, csi i, double beta, double *x) ;
G2O_CSPARSE_API csi cs_ipvec (const csi *p, const double *b, double *x, csi n) ;
G2O_CSPARSE_API csi cs_lsolve (const cs *L, double *x) ;
G2O_CSPARSE_API csi cs_ltsolve (const cs *L, double *x) ;
G2O_CSPARSE_API csn *cs_lu (const cs *A, const css *S, double tol) ;
G2O_CSPARSE_API cs *cs_permute (const cs *A, const csi *pinv, const csi *q, csi values) ;
G2O_CSPARSE_API csi *cs_pinv (const csi *p, csi n) ;
G2O_CSPARSE_API csi cs_pvec (const csi *p, const double *b, double *x, csi n) ;
G2O_CSPARSE_API csn *cs_qr (const cs *A, const css *S) ;
G2O_CSPARSE_API css *cs_schol (csi order, const cs *A) ;
G2O_CSPARSE_API css *cs_sqr (csi order, const cs *A, csi qr) ;
G2O_CSPARSE_API cs *cs_symperm (const cs *A, const csi *pinv, csi values) ;
G2O_CSPARSE_API csi cs_updown (cs *L, csi sigma, const cs *C, const csi *parent) ;
G2O_CSPARSE_API csi cs_usolve (const cs *U, double *x) ;
G2O_CSPARSE_API csi cs_utsolve (const cs *U, double *x) ;
/* utilities */
G2O_CSPARSE_API css *cs_sfree (css *S) ;
G2O_CSPARSE_API csn *cs_nfree (csn *N) ;
G2O_CSPARSE_API csd *cs_dfree (csd *D) ;

/* --- tertiary CSparse routines -------------------------------------------- */
G2O_CSPARSE_API csi *cs_counts (const cs *A, const csi *parent, const csi *post, csi ata) ;
G2O_CSPARSE_API double cs_cumsum (csi *p, csi *c, csi n) ;
G2O_CSPARSE_API csi cs_dfs (csi j, cs *G, csi top, csi *xi, csi *pstack, const csi *pinv) ;
G2O_CSPARSE_API csi cs_ereach (const cs *A, csi k, const csi *parent, csi *s, csi *w) ;
G2O_CSPARSE_API csi *cs_etree (const cs *A, csi ata) ;
G2O_CSPARSE_API csi cs_fkeep (cs *A, csi (*fkeep) (csi, csi, double, void *), void *other) ;
G2O_CSPARSE_API double cs_house (double *x, double *beta, csi n) ;
G2O_CSPARSE_API csi cs_leaf (csi i, csi j, const csi *first, csi *maxfirst, csi *prevleaf,
    csi *ancestor, csi *jleaf) ;
G2O_CSPARSE_API csi *cs_maxtrans (const cs *A, csi seed) ;
G2O_CSPARSE_API csi *cs_post (const csi *parent, csi n) ;
G2O_CSPARSE_API csi *cs_randperm (csi n, csi seed) ;
G2O_CSPARSE_API csi cs_reach (cs *G, const cs *B, csi k, csi *xi, const csi *pinv) ;
G2O_CSPARSE_API csi cs_scatter (const cs *A, csi j, double beta, csi *w, double *x, csi mark,
    cs *C, csi nz) ;
G2O_CSPARSE_API csd *cs_scc (cs *A) ;
G2O_CSPARSE_API csi cs_spsolve (cs *G, const cs *B, csi k, csi *xi, double *x,
    const csi *pinv, csi lo) ;
G2O_CSPARSE_API csi cs_tdfs (csi j, csi k, csi *head, const csi *next, csi *post,
    csi *stack) ;
/* utilities */
G2O_CSPARSE_API csd *cs_dalloc (csi m, csi n) ;
G2O_CSPARSE_API csd *cs_ddone (csd *D, cs *C, void *w, csi ok) ;
G2O_CSPARSE_API cs *cs_done (cs *C, void *w, void *x, csi ok) ;
G2O_CSPARSE_API csi *cs_idone (csi *p, cs *C, void *w, csi ok) ;
G2O_CSPARSE_API csn *cs_ndone (csn *N, cs *C, void *w, void *x, csi ok) ;

#define CS_MAX(a,b) (((a) > (b)) ? (a) : (b))
#define CS_MIN(a,b) (((a) < (b)) ? (a) : (b))
#define CS_FLIP(i) (-(i)-2)
#define CS_UNFLIP(i) (((i) < 0) ? CS_FLIP(i) : (i))
#define CS_MARKED(w,j) (w [j] < 0)
#define CS_MARK(w,j) { w [j] = CS_FLIP (w [j]) ; }
#define CS_CSC(A) (A && (A->nz == -1))
#define CS_TRIPLET(A) (A && (A->nz >= 0))

#ifdef __cplusplus
}
#endif

#endif
