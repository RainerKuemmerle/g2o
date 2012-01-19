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

#ifndef G2O_BATCH_STATS_H_
#define G2O_BATCH_STATS_H_

#include <iostream>

#include "g2o_core_api.h"

namespace g2o {

  /**
   * \brief statistics about the optimization
   */
  struct G2O_CORE_API G2OBatchStatistics {
    G2OBatchStatistics();
    int iteration;                    ///< which iteration
    int numVertices;                  ///< how many vertices are involved
    int numEdges;                     ///< how many edges
    double chi2;                      ///< total chi2

    /** timings **/
    // nonlinear part
    double timeResiduals;             ///< residuals
    double timeLinearize;             ///< jacobians
    double timeQuadraticForm;         ///< construct the quadratic form in the graph
    int levenbergIterations;          ///< number of iterations performed by LM
    // block_solver (constructs Ax=b, plus maybe schur)
    double timeSchurrComplement;      ///< compute schurr complement (0 if not done)

    // linear solver (computes Ax=b);
    double timeSymbolicDecomposition; ///< symbolic decomposition (0 if not done)
    double timeNumericDecomposition;  ///< numeric decomposition  (0 if not done)
    double timeLinearSolution;        ///< total time for solving Ax=b (including detup for schur)
    double timeLinearSolver;          ///< time for solving, excluding Schur setup
    int    iterationsLinearSolver;    ///< iterations of PCG, (0 if not used, i.e., Cholesky)
    double timeUpdate;                ///< time to apply the update
    double timeIteration;             ///< total time;

    double timeMarginals;             ///< computing the inverse elements (solve blocks) and thus the marginal covariances

    // information about the Hessian matrix
    size_t hessianDimension;          ///< rows / cols of the Hessian
    size_t hessianPoseDimension;      ///< dimension of the pose matrix in Schur
    size_t hessianLandmarkDimension;  ///< dimension of the landmark matrix in Schur
    size_t choleskyNNZ;               ///< number of non-zeros in the cholesky factor
  };

  std::ostream& operator<<(std::ostream&, const G2OBatchStatistics&);

  // this is really ugly: global stat variable to write statistics
  extern G2O_CORE_API G2OBatchStatistics * globalStats;
}

#endif
