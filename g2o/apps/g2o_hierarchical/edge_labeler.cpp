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

#include <Eigen/Dense>
#include "edge_labeler.h"
#include "g2o/stuff/unscented.h"

namespace g2o {

  using namespace std;
  using namespace Eigen;

  typedef SigmaPoint<VectorXd> MySigmaPoint;

  EdgeLabeler::EdgeLabeler(SparseOptimizer* optimizer) {
    _optimizer = optimizer;
  }

  int EdgeLabeler::labelEdges(std::set<OptimizableGraph::Edge*>& edges){
    // assume the system is "solved"
    // compute the sparse pattern of the inverse
    std::set<std::pair<int, int> > pattern;
    for (std::set<OptimizableGraph::Edge*>::iterator it=edges.begin(); it!=edges.end(); it++){
      augmentSparsePattern(pattern, *it);
    }


    SparseBlockMatrix<MatrixXd> spInv;

    bool result = computePartialInverse(spInv, pattern);
    //cerr << "partial inverse computed = " << result << endl;
    //cerr << "non zero blocks" << spInv.nonZeroBlocks() << endl;

    if (! result ){
      return -1;
    }
    int count=0;
    for (std::set<OptimizableGraph::Edge*>::iterator it=edges.begin(); it!=edges.end(); it++){
      count += labelEdge(spInv, *it) ? 1 : 0;
    }
    return count;
  }

  void EdgeLabeler::augmentSparsePattern(std::set<std::pair<int, int> >& pattern, OptimizableGraph::Edge* e){
    for (size_t i=0; i<e->vertices().size(); i++){
      const OptimizableGraph::Vertex* v=(const OptimizableGraph::Vertex*) e->vertices()[i];
      int ti=v->hessianIndex();
      if (ti==-1)
	continue;
      for (size_t j=i; j<e->vertices().size(); j++){
	const OptimizableGraph::Vertex* v=(const OptimizableGraph::Vertex*) e->vertices()[j];
	int tj = v->hessianIndex();
	if (tj==-1)
	  continue;
	if(tj<ti)
	  swap(ti,tj);
	pattern.insert(std::make_pair(ti, tj));
      }
    }
  }

  bool EdgeLabeler::computePartialInverse(SparseBlockMatrix<MatrixXd>& spinv, const std::set<std::pair<int,int> >& pattern){
    std::vector<std::pair<int, int> > blockIndices(pattern.size());
    // Why this does not work???
    //std::copy(pattern.begin(),pattern.end(),blockIndices.begin());

    int k=0;
    for(std::set<std::pair<int, int> >::const_iterator it= pattern.begin(); it!=pattern.end(); it++){
      blockIndices[k++]=*it;
    }

    //cerr << "sparse pattern contains " << blockIndices.size() << " blocks" << endl;
    return _optimizer->computeMarginals(spinv, blockIndices);
  }

  bool EdgeLabeler::labelEdge( const SparseBlockMatrix<MatrixXd>& spinv, OptimizableGraph::Edge* e){

    Eigen::Map<MatrixXd> info(e->informationData(), e->dimension(), e->dimension());
    // cerr << "original information matrix" << endl;
    // cerr << info << endl;

    int maxDim=0;
    for (size_t i=0; i<e->vertices().size(); i++){
      const OptimizableGraph::Vertex* v=(const OptimizableGraph::Vertex*) e->vertices()[i];
      int ti=v->hessianIndex();
      if (ti==-1)
	continue;
      maxDim+=v->minimalEstimateDimension();
    }


    //cerr << "maxDim= " << maxDim << endl;
    MatrixXd cov(maxDim, maxDim);
    int cumRow=0;
    for (size_t i=0; i<e->vertices().size(); i++){
      const OptimizableGraph::Vertex* vr=(const OptimizableGraph::Vertex*) e->vertices()[i];
      int ti=vr->hessianIndex();
      if (ti>-1) {
	int cumCol=0;
	for (size_t j=0; j<e->vertices().size(); j++){
	  const OptimizableGraph::Vertex* vc=(const OptimizableGraph::Vertex*) e->vertices()[j];
	  int tj = vc->hessianIndex();
	  if (tj>-1){
	    // cerr << "ti=" << ti << " tj=" << tj
	    //    << " cumRow=" << cumRow << " cumCol=" << cumCol << endl;
	    if (ti<=tj){
	      assert(spinv.block(ti, tj));
	      // cerr << "cblock_ptr" << spinv.block(ti, tj) << endl;
	      // cerr << "cblock.size=" << spinv.block(ti, tj)->rows() << "," << spinv.block(ti, tj)->cols() << endl;
	      // cerr << "cblock" << endl;
	      // cerr << *spinv.block(ti, tj) << endl;
	      cov.block(cumRow, cumCol, vr->minimalEstimateDimension(), vc->minimalEstimateDimension()) =
		*spinv.block(ti, tj);
	    } else {
	      assert(spinv.block(tj, ti));
	      // cerr << "cblock.size=" << spinv.block(tj, ti)->cols() << "," << spinv.block(tj, ti)->rows() << endl;
	      // cerr << "cblock" << endl;
	      // cerr << spinv.block(tj, ti)->transpose() << endl;
	      cov.block(cumRow, cumCol, vr->minimalEstimateDimension(), vc->minimalEstimateDimension()) =
		spinv.block(tj, ti)->transpose();
	    }
	    cumCol += vc->minimalEstimateDimension();
	  }
	}
	cumRow += vr->minimalEstimateDimension();
      }
    }

    // cerr << "covariance assembled" << endl;
    // cerr << cov << endl;
    // now cov contains the aggregate marginals of the state variables in the edge
    VectorXd incMean(maxDim);
    incMean.fill(0);
    std::vector<MySigmaPoint, Eigen::aligned_allocator<MySigmaPoint> > incrementPoints;
    if (! sampleUnscented(incrementPoints, incMean, cov)){
      cerr << "sampleUnscented fail" << endl;
      return false;
    }
    // now determine the zero-error measure by applying the error function of the edge
    // with a zero measurement
    // TODO!!!
    bool smss = e->setMeasurementFromState();
    if (! smss) {
      cerr << "FATAL: Edge::setMeasurementFromState() not implemented" << endl;
    }
    assert(smss && "Edge::setMeasurementFromState() not implemented");

    //std::vector<MySigmaPoint> globalPoints(incrementPoints.size());
    std::vector<MySigmaPoint, Eigen::aligned_allocator<MySigmaPoint> > errorPoints(incrementPoints.size());

    // for each sigma point, project it to the global space, by considering those variables
    // that are involved
    //cerr << "sigma points are extracted, remapping to measurement space" << endl;
    for (size_t i=0; i<incrementPoints.size(); i++) {
      int cumPos=0;
      //VectorXd globalPoint(maxDim);

      // push all the "active" state variables
      for (size_t j=0; j<e->vertices().size(); j++){
        OptimizableGraph::Vertex* vr=(OptimizableGraph::Vertex*) e->vertices()[j];
        int tj=vr->hessianIndex();
        if (tj==-1)
          continue;
        vr->push();
      }

      for (size_t j=0; j<e->vertices().size(); j++){
        OptimizableGraph::Vertex* vr=(OptimizableGraph::Vertex*) e->vertices()[j];
        int tj=vr->hessianIndex();
        if (tj==-1)
          continue;
        vr->oplus(&incrementPoints[i]._sample[cumPos]);
        //assert(vr->getMinimalEstimateData(&globalPoint[cumPos]) && "Vertex::getMinimalEstimateData(...) not implemented");
        cumPos+=vr->minimalEstimateDimension();
      }

      // construct the sigma point in the global space
      // globalPoints[i]._sample=globalPoint;
      // globalPoints[i]._wi=incrementPoints[i]._wi;
      // globalPoints[i]._wp=incrementPoints[i]._wp;

      // construct the sigma point in the error space
      e->computeError();
      Map<VectorXd> errorPoint(e->errorData(),e->dimension());

      errorPoints[i]._sample=errorPoint;
      errorPoints[i]._wi=incrementPoints[i]._wi;
      errorPoints[i]._wp=incrementPoints[i]._wp;

      // pop all the "active" state variables
      for (size_t j=0; j<e->vertices().size(); j++){
        OptimizableGraph::Vertex* vr=(OptimizableGraph::Vertex*) e->vertices()[j];
        int tj=vr->hessianIndex();
        if (tj==-1)
          continue;
        vr->pop();
      }

    }

    // reconstruct the covariance of the error by the sigma points
    MatrixXd errorCov(e->dimension(), e->dimension());
    VectorXd errorMean(e->dimension());
    reconstructGaussian(errorMean, errorCov, errorPoints);
    info=errorCov.inverse();

    // cerr << "remapped information matrix" << endl;
    // cerr << info << endl;
    return true;
  }

}
