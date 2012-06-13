// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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

#ifndef G2O_SPARSE_BLOCK_MATRIX_CCS_H
#define G2O_SPARSE_BLOCK_MATRIX_CCS_H

#include <vector>
#include <cassert>
#include <Eigen/Core>

#include "g2o/config.h"

namespace g2o {

  /**
   * \brief Sparse matrix which uses blocks
   *
   * This class is used as a const view on a SparseBlockMatrix
   * which allows a faster iteration over the elements of the
   * matrix.
   */
  template <class MatrixType>
  class SparseBlockMatrixCCS
  {
    public:
      //! this is the type of the elementary block, it is an Eigen::Matrix.
      typedef MatrixType SparseMatrixBlock;

      //! columns of the matrix
      int cols() const {return _colBlockIndices.size() ? _colBlockIndices.back() : 0;}
      //! rows of the matrix
      int rows() const {return _rowBlockIndices.size() ? _rowBlockIndices.back() : 0;}

      struct RowBlock
      {
        int row;
        MatrixType* block;
        RowBlock() : row(-1), block(0) {}
        RowBlock(int r, MatrixType* b) : row(r), block(b) {}
        bool operator<(const RowBlock& other) const { return row < other.row;}
      };
      typedef std::vector<RowBlock>      SparseColumn;

      SparseBlockMatrixCCS(const std::vector<int>& rowIndices, const std::vector<int>& colIndices) :
        _rowBlockIndices(rowIndices), _colBlockIndices(colIndices)
      {}

      //! how many rows does the block at block-row r has?
      inline int rowsOfBlock(int r) const { return r ? _rowBlockIndices[r] - _rowBlockIndices[r-1] : _rowBlockIndices[0] ; }

      //! how many cols does the block at block-col c has?
      inline int colsOfBlock(int c) const { return c ? _colBlockIndices[c] - _colBlockIndices[c-1] : _colBlockIndices[0]; }

      //! where does the row at block-row r starts?
      inline int rowBaseOfBlock(int r) const { return r ? _rowBlockIndices[r-1] : 0 ; }

      //! where does the col at block-col r starts?
      inline int colBaseOfBlock(int c) const { return c ? _colBlockIndices[c-1] : 0 ; }

      //! the block matrices per block-column
      const std::vector<SparseColumn>& blockCols() const { return _blockCols;}
      std::vector<SparseColumn>& blockCols() { return _blockCols;}

      //! indices of the row blocks
      const std::vector<int>& rowBlockIndices() const { return _rowBlockIndices;}

      //! indices of the column blocks
      const std::vector<int>& colBlockIndices() const { return _colBlockIndices;}

    protected:
      const std::vector<int>& _rowBlockIndices; ///< vector of the indices of the blocks along the rows.
      const std::vector<int>& _colBlockIndices; ///< vector of the indices of the blocks along the cols
      std::vector<SparseColumn> _blockCols;
  };

} //end namespace

#endif
