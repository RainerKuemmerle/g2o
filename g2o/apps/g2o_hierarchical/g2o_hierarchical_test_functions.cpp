// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "edge_creator.h"
#include "edge_labeler.h"
#include "g2o/apps/g2o_cli/dl_wrapper.h"
#include "g2o/apps/g2o_cli/g2o_common.h"
#include "g2o/apps/g2o_cli/output_helper.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/estimate_propagator.h"
#include "g2o/core/factory.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"
#include "g2o/stuff/unscented.h"
#include "star.h"

using std::cerr;
using std::endl;

using MySigmaPoint = g2o::SigmaPoint<g2o::VectorX>;

namespace g2o {

void testMarginals(SparseOptimizer& optimizer) {
  cerr << "Projecting marginals" << endl;
  std::vector<std::pair<int, int> > blockIndices;
  for (const auto& v : optimizer.activeVertices()) {
    if (v->hessianIndex() >= 0) {
      blockIndices.emplace_back(v->hessianIndex(), v->hessianIndex());
    }
    // if (v->hessianIndex()>0){
    //   blockIndices.push_back(make_pair(v->hessianIndex()-1,
    //   v->hessianIndex()));
    // }
  }
  SparseBlockMatrix<MatrixX> spinv;
  if (optimizer.computeMarginals(spinv, blockIndices)) {
    for (const auto& v : optimizer.activeVertices()) {
      cerr << "Vertex id:" << v->id() << endl;
      if (v->hessianIndex() >= 0) {
        cerr << "increments block :" << v->hessianIndex() << ", "
             << v->hessianIndex() << " covariance:" << endl;
        VectorX mean(
            v->minimalEstimateDimension());  // HACK: need to set identity
        mean.fill(0);
        VectorX oldMean(
            v->minimalEstimateDimension());  // HACK: need to set identity
        v->getMinimalEstimateData(oldMean.data());
        MatrixX& cov = *(spinv.block(v->hessianIndex(), v->hessianIndex()));
        std::vector<MySigmaPoint, Eigen::aligned_allocator<MySigmaPoint> > spts;
        cerr << cov << endl;
        if (!sampleUnscented(spts, mean, cov)) continue;

        // now apply the oplus operator to the sigma points,
        // and get the points in the global space
        std::vector<MySigmaPoint, Eigen::aligned_allocator<MySigmaPoint> >
            tspts = spts;

        for (size_t j = 0; j < spts.size(); j++) {
          v->push();
          // cerr << "v_before [" << j << "]" << endl;
          v->getMinimalEstimateData(mean.data());
          // cerr << mean << endl;
          // cerr << "sigma [" << j << "]" << endl;
          // cerr << spts[j]._sample << endl;
          v->oplus(
              VectorX::MapType(spts[j]._sample.data(), spts[j]._sample.size()));
          v->getMinimalEstimateData(mean.data());
          tspts[j]._sample = mean;
          // cerr << "oplus [" << j << "]" << endl;
          // cerr << tspts[j]._sample << endl;
          v->pop();
        }
        MatrixX cov2 = cov;
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

int unscentedTest() {
  MatrixX m = MatrixX(6, 6);
  for (int i = 0; i < 6; i++) {
    for (int j = i; j < 6; j++) {
      m(i, j) = m(j, i) = i * j + 1;
    }
  }
  m += MatrixX::Identity(6, 6);
  cerr << m;
  VectorX mean(6);
  mean.fill(1);

  std::vector<MySigmaPoint, Eigen::aligned_allocator<MySigmaPoint> > spts;
  sampleUnscented(spts, mean, m);
  for (size_t i = 0; i < spts.size(); i++) {
    cerr << "Point " << i << " " << endl
         << "wi=" << spts[i]._wi << " wp=" << spts[i]._wp << " " << endl;
    cerr << spts[i]._sample << endl;
  }

  VectorX recMean(6);
  MatrixX recCov(6, 6);

  reconstructGaussian(recMean, recCov, spts);

  cerr << "recMean" << endl;
  cerr << recMean << endl;

  cerr << "recCov" << endl;
  cerr << recCov << endl;

  return 0;
}

}  // namespace g2o
