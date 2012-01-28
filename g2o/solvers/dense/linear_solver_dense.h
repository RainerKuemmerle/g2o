// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
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

#ifndef G2O_LINEAR_SOLVER_DENSE_H
#define G2O_LINEAR_SOLVER_DENSE_H

#include "g2o/core/linear_solver.h"
#include "g2o/core/batch_stats.h"

#include <vector>
#include <utility>
#include<Eigen/Core>
#include<Eigen/Cholesky>


namespace g2o {

  /**
   * \brief linear solver using PCG, pre-conditioner is block Jacobi
   */
  template <typename MatrixType>
  class LinearSolverDense : public LinearSolver<MatrixType>
  {
    public:
      LinearSolverDense() :
      LinearSolver<MatrixType>()
      {


      }

      virtual ~LinearSolverDense()
      {
      }

      virtual bool init()
      {

        return true;
      }

      bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b)
      {

        int n = A.cols();
        int m = A.cols();

        MatrixXd H(n,m);
        H.setZero();


        int c_idx = 0;


        for (size_t i = 0; i < A.blockCols().size(); ++i)
        {
          int c_size = A.colsOfBlock(i);
          int r_idx = 0;

          const typename SparseBlockMatrix<MatrixType>::IntBlockMap& col
              = A.blockCols()[i];
          if (col.size() > 0)
          {
            typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it;
            for (it = col.begin(); it != col.end(); ++it)
            {

              if (it->first <= (int)i)  // only the upper triangular block is needed
             {
                int r_size = A.rowsOfBlock(it->first);
                H.block(r_idx,c_idx,r_size,c_size)
                    = *(it->second);

                r_idx += r_size;
              }
            }
          }


          c_idx += c_size;
        }

        //std::cerr << H << std::endl;

        Eigen::Map<Eigen::VectorXd> xvec(x, m);
        const Eigen::Map<Eigen::VectorXd> bvec(b, n);

        //std::cerr << bvec << std::endl;
        xvec = H.ldlt().solve(Eigen::VectorXd(bvec));


        //std::cerr << xvec << std::endl;

        return true;
      }



    protected:

  };


}// end namespace

#endif
