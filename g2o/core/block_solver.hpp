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

#include "sparse_optimizer.h"
#include <Eigen/LU>
#include <fstream>
#include <iomanip>

#include "g2o/stuff/timeutil.h"
#include "g2o/stuff/macros.h"

namespace g2o {
  using namespace std;
  using namespace Eigen;

template <typename Traits>
BlockSolver<Traits>::BlockSolver(LinearSolverType* linearSolver) :
  Solver(),
  _linearSolver(linearSolver)
{
  // workspace
  _Hpp=0;
  _Hll=0;
  _Hpl=0;
  _Hschur=0;
  _DInvSchur=0;
  _coefficients=0;
  _bschur = 0;
  _xSize=0;
  _numPoses=0;
  _numLandmarks=0;
  _sizePoses=0;
  _sizeLandmarks=0;
  _doSchur=true;
}

template <typename Traits>
void BlockSolver<Traits>::resize(int* blockPoseIndices, int numPoseBlocks, 
              int* blockLandmarkIndices, int numLandmarkBlocks,
              int s)
{
  deallocate();

  resizeVector(s);

  if (_doSchur) {
    // TODO the following two are only used in schur, actually too large...
    _coefficients = new double [s];
    _bschur = new double[s];
  }

  _Hpp=new PoseHessianType(blockPoseIndices, blockPoseIndices, numPoseBlocks, numPoseBlocks);
  if (_doSchur) {
    _Hschur=new PoseHessianType(blockPoseIndices, blockPoseIndices, numPoseBlocks, numPoseBlocks);
    _Hll=new LandmarkHessianType(blockLandmarkIndices, blockLandmarkIndices, numLandmarkBlocks, numLandmarkBlocks);
    _DInvSchur=new LandmarkHessianType(blockLandmarkIndices, blockLandmarkIndices, numLandmarkBlocks, numLandmarkBlocks);
    _Hpl=new PoseLandmarkHessianType(blockPoseIndices, blockLandmarkIndices, numPoseBlocks, numLandmarkBlocks);
  }
}

template <typename Traits>
void BlockSolver<Traits>::deallocate()
{
  if (_Hpp){
    delete _Hpp;
    _Hpp=0;
  }
  if (_Hll){
    delete _Hll;
    _Hll=0;
  }
  if (_Hpl){
    delete _Hpl;
    _Hpl = 0;
  }
  if (_Hschur){
    delete _Hschur;
    _Hschur=0;
  }
  if (_DInvSchur){
    delete _DInvSchur;
    _DInvSchur=0;
  }
  if (_coefficients) {
    delete[] _coefficients;
    _coefficients = 0;
  }
  if (_bschur) {
    delete[] _bschur;
    _bschur = 0;
  }
}

template <typename Traits>
BlockSolver<Traits>::~BlockSolver()
{
  delete _linearSolver;
  deallocate();
}

template <typename Traits>
bool BlockSolver<Traits>::buildStructure(bool zeroBlocks)
{
  assert(_optimizer);

  size_t sparseDim = 0;
  _numPoses=0;
  _numLandmarks=0;
  _sizePoses=0;
  _sizeLandmarks=0;
#ifdef _MSC_VER
  int* blockPoseIndices = new int[_optimizer->indexMapping().size()];
  int* blockLandmarkIndices = new int[_optimizer->indexMapping().size()];
#else
  int blockPoseIndices[_optimizer->indexMapping().size()];
  int blockLandmarkIndices[_optimizer->indexMapping().size()];
#endif

  for (size_t i = 0; i < _optimizer->indexMapping().size(); ++i) {
    OptimizableGraph::Vertex* v = _optimizer->indexMapping()[i];
    int dim = v->dimension();
    if (! v->marginalized()){
      v->setColInHessian(_sizePoses);
      _sizePoses+=dim;
      blockPoseIndices[_numPoses]=_sizePoses;
      ++_numPoses;
    } else {
      v->setColInHessian(_sizeLandmarks);
      _sizeLandmarks+=dim;
      blockLandmarkIndices[_numLandmarks]=_sizeLandmarks;
      ++_numLandmarks;
    }
    sparseDim += dim;
  }
  resize(blockPoseIndices, _numPoses, blockLandmarkIndices, _numLandmarks, sparseDim);
#ifdef _MSC_VER
  delete[] blockLandmarkIndices;
  delete[] blockPoseIndices;
#endif
  
  // allocate the diagonal on Hpp and Hll
  int poseIdx = 0;
  int landmarkIdx = 0;
  for (size_t i = 0; i < _optimizer->indexMapping().size(); ++i) {
    OptimizableGraph::Vertex* v = _optimizer->indexMapping()[i];
    if (! v->marginalized()){
      //assert(poseIdx == v->hessianIndex());
      PoseMatrixType* m = _Hpp->block(poseIdx, poseIdx, true);
      if (zeroBlocks)
        m->setZero();
      v->mapHessianMemory(m->data());
      ++poseIdx;
    } else {
      LandmarkMatrixType* m = _Hll->block(landmarkIdx, landmarkIdx, true);
      if (zeroBlocks)
        m->setZero();
      v->mapHessianMemory(m->data());
      _DInvSchur->block(landmarkIdx, landmarkIdx, true);
      ++landmarkIdx;
    }
  }
  assert(poseIdx == _numPoses && landmarkIdx == _numLandmarks);

  // here we assume that the landmark indices start after the pose ones
  // create the structure in Hpp, Hll and in Hpl
  for (SparseOptimizer::EdgeContainer::const_iterator it=_optimizer->activeEdges().begin(); it!=_optimizer->activeEdges().end(); ++it){
    OptimizableGraph::Edge* e = *it;

    for (size_t viIdx = 0; viIdx < e->vertices().size(); ++viIdx) {
      OptimizableGraph::Vertex* v1 = (OptimizableGraph::Vertex*) e->vertex(viIdx);
      int ind1 = v1->hessianIndex();
      if (ind1 == -1)
        continue;
      int indexV1Bak = ind1;
      for (size_t vjIdx = viIdx + 1; vjIdx < e->vertices().size(); ++vjIdx) {
        OptimizableGraph::Vertex* v2 = (OptimizableGraph::Vertex*) e->vertex(vjIdx);
        int ind2 = v2->hessianIndex();
        if (ind2 == -1)
          continue;
        ind1 = indexV1Bak;
        bool transposedBlock = ind1 > ind2;
        if (transposedBlock){ // make sure, we allocate the upper triangle block
          swap(ind1, ind2);
        }
        if (! v1->marginalized() && !v2->marginalized()){
          PoseMatrixType* m = _Hpp->block(ind1, ind2, true);
          if (zeroBlocks)
            m->setZero();
          e->mapHessianMemory(m->data(), viIdx, vjIdx, transposedBlock);
          if (_Hschur) // assume this is only needed in case we solve with the schur complement
            _Hschur->block(ind1, ind2, true);
        } else if (v1->marginalized() && v2->marginalized()){
          // RAINER hmm.... should we ever reach this here????
          LandmarkMatrixType* m = _Hll->block(ind1-_numPoses, ind2-_numPoses, true);
          if (zeroBlocks)
            m->setZero();
          e->mapHessianMemory(m->data(), viIdx, vjIdx, false);
        } else { 
          if (v1->marginalized()){ 
            PoseLandmarkMatrixType* m = _Hpl->block(v2->hessianIndex(),v1->hessianIndex()-_numPoses, true);
            if (zeroBlocks)
              m->setZero();
            e->mapHessianMemory(m->data(), viIdx, vjIdx, true); // transpose the block before writing to it
          } else {
            PoseLandmarkMatrixType* m = _Hpl->block(v1->hessianIndex(),v2->hessianIndex()-_numPoses, true);
            if (zeroBlocks)
              m->setZero();
            e->mapHessianMemory(m->data(), viIdx, vjIdx, false); // directly the block
          }
        }
      }
    }
  }

  if (! _doSchur)
    return true;
  // allocate the blocks for the schur complement
  for (size_t i = 0; i < _optimizer->indexMapping().size(); ++i) {
    OptimizableGraph::Vertex* v = _optimizer->indexMapping()[i];
    if (v->marginalized()){
      const HyperGraph::EdgeSet& vedges=v->edges();
      for (HyperGraph::EdgeSet::const_iterator it1=vedges.begin(); it1!=vedges.end(); ++it1){
        if ((*it1)->vertices().size() != 2)
          continue;
        OptimizableGraph::Vertex* v1= (OptimizableGraph::Vertex*) (*it1)->vertex(0);
        if (v1==v)
          v1 = (OptimizableGraph::Vertex*) (*it1)->vertex(1);
        if (v1->hessianIndex()==-1)
          continue;
        for  (HyperGraph::EdgeSet::const_iterator it2=vedges.begin(); it2!=vedges.end(); ++it2){
          if ((*it2)->vertices().size() != 2)
            continue;
          OptimizableGraph::Vertex* v2= (OptimizableGraph::Vertex*) (*it2)->vertex(0);
          if (v2==v)
            v2 = (OptimizableGraph::Vertex*) (*it2)->vertex(1);
          if (v2->hessianIndex()==-1)
            continue;
          int i1=v1->hessianIndex();
          int i2=v2->hessianIndex();
          if (i1<=i2)
            _Hschur->block(i1,i2,true)->setZero();
        }
      }
    }
  }

  return true;
}

template <typename Traits>
bool BlockSolver<Traits>::updateStructure(const std::vector<HyperGraph::Vertex*>& vset, const HyperGraph::EdgeSet& edges)
{
  for (std::vector<HyperGraph::Vertex*>::const_iterator vit = vset.begin(); vit != vset.end(); ++vit) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*vit);
    int dim = v->dimension();
    if (! v->marginalized()){
      v->setColInHessian(_sizePoses);
      _sizePoses+=dim;
      _Hpp->rowBlockIndices().push_back(_sizePoses);
      _Hpp->colBlockIndices().push_back(_sizePoses);
      _Hpp->blockCols().push_back(typename SparseBlockMatrix<PoseMatrixType>::IntBlockMap());
      ++_numPoses;
      int ind = v->hessianIndex();
      PoseMatrixType* m = _Hpp->block(ind, ind, true);
      v->mapHessianMemory(m->data());
    } else {
      std::cerr << "updateStructure(): Schur not supported" << std::endl;
      abort();
    }
  }
  resizeVector(_sizePoses + _sizeLandmarks);

  for (HyperGraph::EdgeSet::const_iterator it = edges.begin(); it != edges.end(); ++it) {
    OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);

    for (size_t viIdx = 0; viIdx < e->vertices().size(); ++viIdx) {
      OptimizableGraph::Vertex* v1 = (OptimizableGraph::Vertex*) e->vertex(viIdx);
      int ind1 = v1->hessianIndex();
      int indexV1Bak = ind1;
      if (ind1 == -1)
        continue;
      for (size_t vjIdx = viIdx + 1; vjIdx < e->vertices().size(); ++vjIdx) {
        OptimizableGraph::Vertex* v2 = (OptimizableGraph::Vertex*) e->vertex(vjIdx);
        int ind2 = v2->hessianIndex();
        if (ind2 == -1)
          continue;
        ind1 = indexV1Bak;
        bool transposedBlock = ind1 > ind2;
        if (transposedBlock) // make sure, we allocate the upper triangular block
          swap(ind1, ind2);

        if (! v1->marginalized() && !v2->marginalized()) {
          PoseMatrixType* m = _Hpp->block(ind1, ind2, true);
          e->mapHessianMemory(m->data(), viIdx, vjIdx, transposedBlock);
        } else { 
          std::cerr << __PRETTY_FUNCTION__ << ": not supported" << std::endl;
        }
      }
    }

  }

  return true;
}

template <typename Traits>
bool BlockSolver<Traits>::solve(){
  //cerr << __PRETTY_FUNCTION__ << endl;
  if (! _doSchur){
    double t=get_time();
    bool ok = _linearSolver->solve(*_Hpp, _x, _b);
    if (globalStats) {
      globalStats->timeLinearSolver = get_time() - t;
      globalStats->hessianDimension = globalStats->hessianPoseDimension = _Hpp->cols();
    }
    return ok;
  }

  // schur thing

  // backup the coefficient matrix
  double t=get_time();
  _Hschur->clear();
  _Hpp->add(_Hschur);
  _DInvSchur->clear();
  memset (_coefficients, 0, _xSize*sizeof(double));
  for (size_t i = 0; i < _optimizer->indexMapping().size(); ++i) {
    OptimizableGraph::Vertex* v = _optimizer->indexMapping()[i];
    if (v->marginalized()){
      int landmarkIndex=i-_numPoses;
      const HyperGraph::EdgeSet& vedges=v->edges();

      const LandmarkMatrixType * D=_Hll->block(landmarkIndex,landmarkIndex);
      assert (D);
      assert (D->rows()==D->cols());
      LandmarkMatrixType Dinv=D->inverse();
      LandmarkMatrixType * _DInvSchurBlock=_DInvSchur->block(landmarkIndex, landmarkIndex, false);
      assert(_DInvSchurBlock);
      assert(_DInvSchurBlock->rows()==D->rows());
      assert(_DInvSchurBlock->cols()==D->cols());

      *_DInvSchurBlock=Dinv;
      LandmarkVectorType  db(D->rows());
      for (int j=0; j<D->rows(); ++j) {
        db[j]=_b[v->colInHessian()+_sizePoses+j];
      }
      db=Dinv*db;

      // helper array for OpenMP parallel
      size_t tmpIdx = 0;
#     ifdef _MSC_VER
      OptimizableGraph::Edge** tmpEdges = new OptimizableGraph::Edge*[vedges.size()];
#     else
      OptimizableGraph::Edge* tmpEdges[vedges.size()];
#     endif
      for (HyperGraph::EdgeSet::const_iterator it2=vedges.begin(); it2!=vedges.end(); ++it2) {
        OptimizableGraph::Edge* e2 = static_cast<OptimizableGraph::Edge*>(*it2);
        tmpEdges[tmpIdx++] = e2;
      }

#     ifdef G2O_OPENMP
#     pragma omp parallel for default (shared)
#     endif
      for (size_t l=0; l < tmpIdx; ++l) {
        OptimizableGraph::Edge* e1 = tmpEdges[l];
        if (e1->vertices().size() != 2)
          continue;
        OptimizableGraph::Vertex* v1= static_cast<OptimizableGraph::Vertex*>( e1->vertex(0) );
        if (v1==v)
          v1 = (OptimizableGraph::Vertex*) e1->vertex(1);

        assert (!v1->marginalized());
        int i1=v1->hessianIndex();
        if (i1<0)
          continue;

        const PoseLandmarkMatrixType* Bi=_Hpl->block(i1,landmarkIndex);
        assert(Bi);

        PoseVectorType Bb=(*Bi)*db;
        v1->lockQuadraticForm();
        for (int j=0; j<Bb.rows(); ++j){
          _coefficients[v1->colInHessian()+j]+=Bb(j);
        }
        PoseLandmarkMatrixType BDinv = (*Bi)*(Dinv);

        for (size_t k =0; k < tmpIdx; ++k) {
          OptimizableGraph::Edge* e2 = tmpEdges[k];
          if (e2->vertices().size() != 2)
            continue;
          OptimizableGraph::Vertex* v2= (OptimizableGraph::Vertex*) e2->vertex(0);
          if (v2==v)
            v2 = (OptimizableGraph::Vertex*) e2->vertex(1);

          assert (!v2->marginalized());
          int i2=v2->hessianIndex();
          if (i2<0)
            continue;
          if (i1>i2)
            continue;

          const PoseLandmarkMatrixType* Bj = _Hpl->block(i2,landmarkIndex);
          assert(Bj); 

          PoseMatrixType* Hi1i2 = _Hschur->block(i1,i2);
          assert(Hi1i2);
          (*Hi1i2).noalias() -= BDinv*Bj->transpose();
        }
        v1->unlockQuadraticForm();
      }
#     ifdef _MSC_VER
      delete[] tmpEdges;
#     endif
    }
  }
  //cerr << "Solve [marginalize] = " <<  get_time()-t << endl;

  // _bschur = _b for calling solver, and not touching _b
  memcpy(_bschur, _b, _xSize * sizeof(double));
  for (int i=0; i<_sizePoses; ++i){
    _bschur[i]-=_coefficients[i];
  }

  if (globalStats){
    globalStats->timeSchurrComplement = get_time() - t;
  }

  t=get_time();
  bool solvedPoses = _linearSolver->solve(*_Hschur, _x, _bschur);
  if (globalStats) {
    globalStats->timeLinearSolver = get_time() - t;
    globalStats->hessianPoseDimension = _Hpp->cols();
    globalStats->hessianLandmarkDimension = _Hll->cols();
    globalStats->hessianDimension = globalStats->hessianPoseDimension + globalStats->hessianLandmarkDimension;
  }
  //cerr << "Solve [decompose and solve] = " <<  get_time()-t << endl;

  if (! solvedPoses)
    return false;

  // _x contains the solution for the poses, now applying it to the landmarks to get the new part of the
  // solution;
  double* xp = _x;
  double* cp = _coefficients;

  double* xl=_x+_sizePoses;
  double* cl=_coefficients + _sizePoses;
  double* bl=_b+_sizePoses;

  // cp = -xp
  for (int i=0; i<_sizePoses; ++i)
    cp[i]=-xp[i];

  // cl = bl
  memcpy(cl,bl,_sizeLandmarks*sizeof(double));

  // cl = bl - Bt * xp
  //Bt->multiply(cl, cp);
  _Hpl->rightMultiply(cl, cp);

  // xl = Dinv * cl
  memset(xl,0, _sizeLandmarks*sizeof(double));
  //_DInvSchur->multiply(xl,cl);
  _DInvSchur->rightMultiply(xl,cl);
  //cerr << "Solve [landmark delta] = " <<  get_time()-t << endl;

  return true;
}


template <typename Traits>
bool BlockSolver<Traits>::computeMarginals(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<std::pair<int, int> >& blockIndices)
{
  double t = get_time();
  bool ok = _linearSolver->solvePattern(spinv, blockIndices, *_Hpp);
  if (globalStats) {
    globalStats->timeMarginals = get_time() - t;
  }
  return ok;
}

template <typename Traits>
bool BlockSolver<Traits>::buildSystem()
{
  // clear b vector
# ifdef G2O_OPENMP
# pragma omp parallel for default (shared) if (_optimizer->indexMapping().size() > 1000)
# endif
  for (int i = 0; i < static_cast<int>(_optimizer->indexMapping().size()); ++i) {
    OptimizableGraph::Vertex* v=_optimizer->indexMapping()[i];
    assert(v);
    v->clearQuadraticForm();
  }
  _Hpp->clear();
  if (_doSchur) {
    _Hll->clear();
    _Hpl->clear();
  }

  // resetting the terms for the pairwise constraints
  // built up the current system by storing the Hessian blocks in the edges and vertices
# ifdef G2O_OPENMP
# pragma omp parallel for default (shared) if (_optimizer->activeEdges().size() > 100)
# endif
  for (int k = 0; k < static_cast<int>(_optimizer->activeEdges().size()); ++k) {
    OptimizableGraph::Edge* e = _optimizer->activeEdges()[k];
    e->constructQuadraticForm();
  }

  // flush the current system in a sparse block matrix
# ifdef G2O_OPENMP
# pragma omp parallel for default (shared) if (_optimizer->indexMapping().size() > 1000)
# endif
  for (int i = 0; i < static_cast<int>(_optimizer->indexMapping().size()); ++i) {
    OptimizableGraph::Vertex* v=_optimizer->indexMapping()[i];
    int iBase = v->colInHessian();
    if (v->marginalized())
      iBase+=_sizePoses;
    v->copyB(_b+iBase);
  }

  return 0;
}


template <typename Traits>
bool BlockSolver<Traits>::setLambda(double lambda)
{
# ifdef G2O_OPENMP
# pragma omp parallel for default (shared) if (_numPoses > 100)
# endif
  for (int i = 0; i < _numPoses; ++i) {
    PoseMatrixType *b=_Hpp->block(i,i);
    b->diagonal().array() += lambda;
  }
# ifdef G2O_OPENMP
# pragma omp parallel for default (shared) if (_numLandmarks > 100)
# endif
  for (int i = 0; i < _numLandmarks; ++i) {
    LandmarkMatrixType *b=_Hll->block(i,i);
    b->diagonal().array() += lambda;
  }
  return true;
}

template <typename Traits>
bool BlockSolver<Traits>::init(SparseOptimizer* optimizer, bool online)
{
  _optimizer = optimizer;
  if (! online) {
    if (_Hpp)
      _Hpp->clear();
    if (_Hpl)
      _Hpl->clear();
    if (_Hll)
      _Hll->clear();
  }
  _linearSolver->init();
  return true;
}

template <typename Traits>
void BlockSolver<Traits>::setWriteDebug(bool writeDebug)
{
  _linearSolver->setWriteDebug(writeDebug);
}

template <typename Traits>
bool BlockSolver<Traits>::saveHessian(const std::string& fileName) const
{
  return _Hpp->writeOctave(fileName.c_str(), true);
}

} // end namespace
