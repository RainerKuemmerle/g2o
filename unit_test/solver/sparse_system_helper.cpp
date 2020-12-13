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

#include "sparse_system_helper.h"

#include <stdexcept>
#include <vector>

#include "g2o/core/io_helper.h"

namespace g2o {
namespace internal {

std::vector<int> readIndeces(const std::string& indeces) {
  std::stringstream tokens(indeces);

  int numElems = 0;
  tokens >> numElems;
  std::vector<int> result;
  for (int i = 0; i < numElems; ++i) {
    int number;
    tokens >> number;
    result.push_back(number);
  }
  return result;
}

// return the serialized sparse matrix
std::string sparseMatrixString() {
  std::stringstream aux;
  aux << "RBI : 12 3 6 9 12 15 18 21 24 27 30 33 36" << std::endl;
  aux << "CBI : 12 3 6 9 12 15 18 21 24 27 30 33 36" << std::endl;
  aux << "BLOCK : 0 0" << std::endl;
  aux << "2500 1.79023e-15 0" << std::endl;
  aux << "-1.79023e-15 2500 0" << std::endl;
  aux << "0 0 25000" << std::endl;
  aux << "BLOCK : 1 1" << std::endl;
  aux << "500 0 0" << std::endl;
  aux << "0 500 0" << std::endl;
  aux << "0 0 5000" << std::endl;
  aux << "BLOCK : 2 2" << std::endl;
  aux << "500 8.88178e-16 0" << std::endl;
  aux << "-8.88178e-16 500 0" << std::endl;
  aux << "0 0 5000" << std::endl;
  aux << "BLOCK : 3 3" << std::endl;
  aux << "500 4.44089e-16 0" << std::endl;
  aux << "-4.44089e-16 500 0" << std::endl;
  aux << "0 0 5000" << std::endl;
  aux << "BLOCK : 4 4" << std::endl;
  aux << "500 0 0" << std::endl;
  aux << "0 500 0" << std::endl;
  aux << "0 0 5000" << std::endl;
  aux << "BLOCK : 5 5" << std::endl;
  aux << "500 0 0" << std::endl;
  aux << "0 500 0" << std::endl;
  aux << "0 0 5000" << std::endl;
  aux << "BLOCK : 6 6" << std::endl;
  aux << "500 -8.88178e-16 0" << std::endl;
  aux << "8.88178e-16 500 0" << std::endl;
  aux << "0 0 5000" << std::endl;
  aux << "BLOCK : 0 7" << std::endl;
  aux << "-500 0 -2000" << std::endl;
  aux << "0 -500 -500" << std::endl;
  aux << "0 0 -5000" << std::endl;
  aux << "BLOCK : 7 7" << std::endl;
  aux << "500 0 2000" << std::endl;
  aux << "0 500 500" << std::endl;
  aux << "2000 500 13500" << std::endl;
  aux << "BLOCK : 0 8" << std::endl;
  aux << "-500 0 -1500" << std::endl;
  aux << "0 -500 -500" << std::endl;
  aux << "0 0 -5000" << std::endl;
  aux << "BLOCK : 8 8" << std::endl;
  aux << "500 0 1500" << std::endl;
  aux << "0 500 500" << std::endl;
  aux << "1500 500 10000" << std::endl;
  aux << "BLOCK : 0 9" << std::endl;
  aux << "-500 1.38778e-17 -1000" << std::endl;
  aux << "-1.38778e-17 -500 -500" << std::endl;
  aux << "0 0 -5000" << std::endl;
  aux << "BLOCK : 9 9" << std::endl;
  aux << "500 1.38778e-17 1000" << std::endl;
  aux << "-1.38778e-17 500 500" << std::endl;
  aux << "1000 500 7500" << std::endl;
  aux << "BLOCK : 0 10" << std::endl;
  aux << "-500 0 -500" << std::endl;
  aux << "0 -500 -500" << std::endl;
  aux << "0 0 -5000" << std::endl;
  aux << "BLOCK : 10 10" << std::endl;
  aux << "500 0 500" << std::endl;
  aux << "0 500 500" << std::endl;
  aux << "500 500 6000" << std::endl;
  aux << "BLOCK : 0 11" << std::endl;
  aux << "-500 1.77636e-15 -500" << std::endl;
  aux << "-1.77636e-15 -500 -500" << std::endl;
  aux << "0 0 -5000" << std::endl;
  aux << "BLOCK : 1 11" << std::endl;
  aux << "-500 0 -500" << std::endl;
  aux << "0 -500 -500" << std::endl;
  aux << "0 0 -5000" << std::endl;
  aux << "BLOCK : 2 11" << std::endl;
  aux << "-500 8.88178e-16 8.88178e-16" << std::endl;
  aux << "-8.88178e-16 -500 -500" << std::endl;
  aux << "0 0 -5000" << std::endl;
  aux << "BLOCK : 3 11" << std::endl;
  aux << "-500 4.44089e-16 500" << std::endl;
  aux << "-4.44089e-16 -500 -500" << std::endl;
  aux << "0 0 -5000" << std::endl;
  aux << "BLOCK : 4 11" << std::endl;
  aux << "-500 0 1000" << std::endl;
  aux << "0 -500 -500" << std::endl;
  aux << "0 0 -5000" << std::endl;
  aux << "BLOCK : 5 11" << std::endl;
  aux << "-500 0 1500" << std::endl;
  aux << "0 -500 -500" << std::endl;
  aux << "0 0 -5000" << std::endl;
  aux << "BLOCK : 6 11" << std::endl;
  aux << "-500 -8.88178e-16 1000" << std::endl;
  aux << "8.88178e-16 -500 -500" << std::endl;
  aux << "0 0 -5000" << std::endl;
  aux << "BLOCK : 11 11" << std::endl;
  aux << "4000 2.22045e-15 -3500" << std::endl;
  aux << "-2.22045e-15 4000 4000" << std::endl;
  aux << "-3500 4000 54500" << std::endl;
  return aux.str();
}

g2o::SparseBlockMatrixX createTestMatrix() {
  std::stringstream input(sparseMatrixString());
  std::string token;

  // reading RBI
  input >> token >> token;
  std::getline(input, token);
  std::vector<int> rowBlockIndeces = readIndeces(token);
  // reading CBI
  input >> token >> token;
  std::getline(input, token);
  std::vector<int> colBlockIndeces = readIndeces(token);

  // allocating the matrix and read the elements of the matrix
  g2o::SparseBlockMatrixX result(rowBlockIndeces.data(), colBlockIndeces.data(),
                                 rowBlockIndeces.size(), colBlockIndeces.size());
  while (input >> token) {
    if (token != "BLOCK") {
      throw std::logic_error("Expected BLOCK as token");
    }
    input >> token;
    int ri, ci;
    input >> ri >> ci;
    auto sparseBlock = result.block(ri, ci, true);
    sparseBlock->resize(result.rowsOfBlock(ri), result.colsOfBlock(ci));
    for (int rr = 0; rr < sparseBlock->rows(); rr++) {
      for (int cc = 0; cc < sparseBlock->cols(); cc++) {
        double value;
        input >> value;
        (*sparseBlock)(rr, cc) = value;
      }
    }
  }
  return result;
}

g2o::VectorX createTestVectorB() {
  g2o::VectorX result;
  result.resize(12 * 3);
  int idx = 0;
  result(idx++) = 6.19602;
  result(idx++) = -53.8323;
  result(idx++) = 206.224;
  result(idx++) = 57.225;
  result(idx++) = -13.98;
  result(idx++) = -58.65;
  result(idx++) = 19.87;
  result(idx++) = -12.0698;
  result(idx++) = -44.5;
  result(idx++) = -7.5935;
  result(idx++) = 22.79;
  result(idx++) = -34.15;
  result(idx++) = 2.71;
  result(idx++) = 47.155;
  result(idx++) = -20.35;
  result(idx++) = -23.688;
  result(idx++) = 8.295;
  result(idx++) = -162.7;
  result(idx++) = -1.376;
  result(idx++) = -32.535;
  result(idx++) = 65.45;
  result(idx++) = -3.00264;
  result(idx++) = 3.27685;
  result(idx++) = -41.2837;
  result(idx++) = 41.851;
  result(idx++) = -3.65832;
  result(idx++) = 137.045;
  result(idx++) = -6.67126;
  result(idx++) = 18.5169;
  result(idx++) = 4.47434;
  result(idx++) = -23.5981;
  result(idx++) = 12.2319;
  result(idx++) = -94.4162;
  result(idx++) = -95.8125;
  result(idx++) = 34.0934;
  result(idx++) = 87.0203;
  return result;
}

g2o::VectorX createTestVectorX() {
  g2o::VectorX result;
  result.resize(12 * 3);
  int idx = 0;
  result(idx++) = -0.03134963;
  result(idx++) = 0.01363732;
  result(idx++) = 0.02445500;
  result(idx++) = 0.05355033;
  result(idx++) = 0.03260726;
  result(idx++) = -0.00828985;
  result(idx++) = -0.02459981;
  result(idx++) = 0.03642766;
  result(idx++) = -0.00545985;
  result(idx++) = -0.08296696;
  result(idx++) = 0.10614726;
  result(idx++) = -0.00338985;
  result(idx++) = -0.06580011;
  result(idx++) = 0.15487726;
  result(idx++) = -0.00062985;
  result(idx++) = -0.12203625;
  result(idx++) = 0.07715726;
  result(idx++) = -0.02909985;
  result(idx++) = -0.07397211;
  result(idx++) = -0.00450274;
  result(idx++) = 0.01653015;
  result(idx++) = -0.10913493;
  result(idx++) = 0.00224601;
  result(idx++) = 0.01794501;
  result(idx++) = -0.03010283;
  result(idx++) = -0.02116439;
  result(idx++) = 0.02748507;
  result(idx++) = -0.09332214;
  result(idx++) = 0.02635612;
  result(idx++) = 0.02431500;
  result(idx++) = -0.08639083;
  result(idx++) = 0.03025612;
  result(idx++) = 0.00784500;
  result(idx++) = -0.06433981;
  result(idx++) = 0.05712711;
  result(idx++) = 0.00344015;
  return result;
}

}  // namespace internal
}  // namespace g2o
