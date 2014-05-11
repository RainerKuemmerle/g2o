#include <signal.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cassert>
#include <sstream>
#include "g2o/apps/g2o_cli/dl_wrapper.h"
#include "g2o/apps/g2o_cli/output_helper.h"
#include "g2o/apps/g2o_cli/g2o_common.h"

#include "g2o/core/estimate_propagator.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/hyper_dijkstra.h"

#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"

#include "edge_labeler.h"
#include "edge_creator.h"
#include "star.h"

#include "g2o/stuff/unscented.h"
#include <Eigen/Core>
#include <Eigen/Dense>


using namespace std;
using namespace g2o;
using namespace Eigen;

typedef SigmaPoint<VectorXd> MySigmaPoint;

void testMarginals(SparseOptimizer& optimizer){
  cerr << "Projecting marginals" << endl;
  std::vector<std::pair<int, int> > blockIndices;
  for (size_t i=0; i<optimizer.activeVertices().size(); i++) {
    OptimizableGraph::Vertex* v=optimizer.activeVertices()[i];
    if (v->hessianIndex()>=0){
      blockIndices.push_back(make_pair(v->hessianIndex(), v->hessianIndex()));
    }
    // if (v->hessianIndex()>0){
    //   blockIndices.push_back(make_pair(v->hessianIndex()-1, v->hessianIndex()));
    // }
  }
  SparseBlockMatrix<MatrixXd> spinv;
  if (optimizer.computeMarginals(spinv, blockIndices)) {
    for (size_t i=0; i<optimizer.activeVertices().size(); i++) {
      OptimizableGraph::Vertex* v=optimizer.activeVertices()[i];
      cerr << "Vertex id:" << v->id() << endl;
      if (v->hessianIndex()>=0){
        cerr << "increments block :" << v->hessianIndex() << ", " << v->hessianIndex()<< " covariance:" <<  endl;
        VectorXd mean(v->minimalEstimateDimension()); //HACK: need to set identity
        mean.fill(0);
        VectorXd oldMean(v->minimalEstimateDimension()); //HACK: need to set identity
        v->getMinimalEstimateData(&oldMean[0]);
        MatrixXd& cov= *(spinv.block(v->hessianIndex(), v->hessianIndex()));
        std::vector<MySigmaPoint, Eigen::aligned_allocator<MySigmaPoint> > spts;
        cerr << cov << endl;
        if (! sampleUnscented(spts,mean,cov) )
          continue;

        // now apply the oplus operator to the sigma points,
        // and get the points in the global space
        std::vector<MySigmaPoint, Eigen::aligned_allocator<MySigmaPoint> > tspts = spts;

        for (size_t j=0; j<spts.size(); j++) {
          v->push();
          // cerr << "v_before [" << j << "]" << endl;
          v->getMinimalEstimateData(&mean[0]);
          // cerr << mean << endl;
          // cerr << "sigma [" << j << "]" << endl;
          // cerr << spts[j]._sample << endl;
          v->oplus(&(spts[j]._sample[0]));
          v->getMinimalEstimateData(&mean[0]);
          tspts[j]._sample=mean;
          // cerr << "oplus [" << j << "]" << endl;
          // cerr << tspts[j]._sample << endl;
          v->pop();
        }
        MatrixXd cov2=cov;
        reconstructGaussian(mean, cov2, tspts);
        cerr << "global block :" << v->hessianIndex() << ", " << v->hessianIndex()<< endl;
        cerr << "mean: " << endl;
        cerr <<  mean << endl;
        cerr << "oldMean: " << endl;
        cerr <<  oldMean << endl;
        cerr << "cov: " << endl;
        cerr << cov2 << endl;

      }
      // if (v->hessianIndex()>0){
      //   cerr << "inv block :" << v->hessianIndex()-1 << ", " << v->hessianIndex()<< endl;
      //   cerr << *(spinv.block(v->hessianIndex()-1, v->hessianIndex()));
      //   cerr << endl;
      // }
    }
  }
}

int unscentedTest(){
  MatrixXd m=MatrixXd(6,6);
  for (int i=0; i<6; i++){
    for (int j=i; j<6; j++){
      m(i,j)=m(j,i)=i*j+1;
    }
  }
  m+=MatrixXd::Identity(6,6);
  cerr << m;
  VectorXd mean(6);
  mean.fill(1);

  std::vector<MySigmaPoint, Eigen::aligned_allocator<MySigmaPoint> > spts;
  sampleUnscented(spts,mean,m);
  for (size_t i =0; i<spts.size(); i++){
    cerr << "Point " << i << " " << endl << "wi=" << spts[i]._wi << " wp=" << spts[i]._wp << " " << endl;
    cerr << spts[i]._sample << endl;
  }

  VectorXd recMean(6);
  MatrixXd recCov(6,6);

  reconstructGaussian(recMean, recCov, spts);

  cerr << "recMean" << endl;
  cerr << recMean << endl;

  cerr << "recCov" << endl;
  cerr << recCov << endl;

  return 0;
}
