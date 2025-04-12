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

#include <gmock/gmock.h>

#include "g2o/core/eigen_types.h"
#include "g2o/stuff/unscented.h"
#include "gmock/gmock.h"
#include "unit_test/test_helper/eigen_matcher.h"

namespace {
using MySigmaPoint = g2o::SigmaPoint<g2o::VectorX>;
}

TEST(Unscented, SampleUnscented) {
  constexpr int kDim = 6;

  g2o::MatrixX covariance = g2o::MatrixX::Zero(kDim, kDim);
  for (int i = 0; i < kDim; i++) {
    for (int j = i; j < kDim; j++) {
      covariance(i, j) = covariance(j, i) = i * j + 1;
    }
  }
  covariance += g2o::MatrixX::Identity(kDim, kDim);
  g2o::VectorX mean = g2o::VectorX::Ones(kDim);

  std::vector<MySigmaPoint> spts;
  sampleUnscented(spts, mean, covariance);
  EXPECT_THAT(spts, testing::SizeIs(2 * kDim + 1));

  g2o::VectorX rec_mean(kDim);
  g2o::MatrixX rec_covariance(kDim, kDim);
  reconstructGaussian(rec_mean, rec_covariance, spts);

  EXPECT_THAT(print_wrap(mean), EigenApproxEqual(print_wrap(rec_mean), 1e-6));
  EXPECT_THAT(print_wrap(covariance),
              EigenApproxEqual(print_wrap(rec_covariance), 1e-6));
}

#if 0
void testMarginals(SparseOptimizer& optimizer) {
  cerr << "Projecting marginals" << endl;
  std::vector<std::pair<int, int> > blockIndices;
  for (size_t i = 0; i < optimizer.activeVertices().size(); i++) {
    OptimizableGraph::Vertex* v = optimizer.activeVertices()[i];
    if (v->hessianIndex() >= 0) {
      blockIndices.push_back(make_pair(v->hessianIndex(), v->hessianIndex()));
    }
    // if (v->hessianIndex()>0){
    //   blockIndices.push_back(make_pair(v->hessianIndex()-1,
    //   v->hessianIndex()));
    // }
  }
  SparseBlockMatrix<MatrixXd> spinv;
  if (optimizer.computeMarginals(spinv, blockIndices)) {
    for (size_t i = 0; i < optimizer.activeVertices().size(); i++) {
      OptimizableGraph::Vertex* v = optimizer.activeVertices()[i];
      cerr << "Vertex id:" << v->id() << endl;
      if (v->hessianIndex() >= 0) {
        cerr << "increments block :" << v->hessianIndex() << ", "
             << v->hessianIndex() << " covariance:" << endl;
        VectorXd mean(
            v->minimalEstimateDimension());  // HACK: need to set identity
        mean.fill(0);
        VectorXd oldMean(
            v->minimalEstimateDimension());  // HACK: need to set identity
        v->getMinimalEstimateData(&oldMean[0]);
        MatrixXd& cov = *(spinv.block(v->hessianIndex(), v->hessianIndex()));
        std::vector<MySigmaPoint> spts;
        cerr << cov << endl;
        if (!sampleUnscented(spts, mean, cov)) continue;

        // now apply the oplus operator to the sigma points,
        // and get the points in the global space
        std::vector<MySigmaPoint> tspts = spts;

        for (size_t j = 0; j < spts.size(); j++) {
          v->push();
          // cerr << "v_before [" << j << "]" << endl;
          v->getMinimalEstimateData(&mean[0]);
          // cerr << mean << endl;
          // cerr << "sigma [" << j << "]" << endl;
          // cerr << spts[j]._sample << endl;
          v->oplus(&(spts[j]._sample[0]));
          v->getMinimalEstimateData(&mean[0]);
          tspts[j]._sample = mean;
          // cerr << "oplus [" << j << "]" << endl;
          // cerr << tspts[j]._sample << endl;
          v->pop();
        }
        MatrixXd cov2 = cov;
        reconstructGaussian(mean, cov2, tspts);
        cerr << "global block :" << v->hessianIndex() << ", "
             << v->hessianIndex() << endl;
        cerr << "mean: " << endl;
        cerr << mean << endl;
        cerr << "oldMean: " << endl;
        cerr << oldMean << endl;
        cerr << "cov: " << endl;
        cerr << cov2 << endl;
      }
      // if (v->hessianIndex()>0){
      //   cerr << "inv block :" << v->hessianIndex()-1 << ", " <<
      //   v->hessianIndex()<< endl; cerr << *(spinv.block(v->hessianIndex()-1,
      //   v->hessianIndex())); cerr << endl;
      // }
    }
  }
}
#endif
