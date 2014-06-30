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

#include "g2o_slam_interface.h"

#include "fast_output.h"

#include "types_slam2d_online.h"
#include "types_slam3d_online.h"

#include "graph_optimizer_sparse_online.h"
#include "g2o/types/slam3d/se3quat.h"

#include <iostream>
using namespace std;

namespace g2o {

  namespace {
    void quat_to_euler(const Eigen::Quaterniond& q, double& yaw, double& pitch, double& roll)
    {
      const double& q0 = q.w();
      const double& q1 = q.x();
      const double& q2 = q.y();
      const double& q3 = q.z();
      roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
      pitch = asin(2*(q0*q2-q3*q1));
      yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
    }

    void jac_quat3_euler3(Eigen::Matrix<double, 6, 6>& J, const SE3Quat& t)
    {
      const Vector3d& tr0 = t.translation();
      const Quaterniond& q0 = t.rotation();

      double delta=1e-6;
      double idelta= 1. / (2. * delta);

      for (int i=0; i<6; i++){
        SE3Quat ta, tb;
        if (i<3){
          Vector3d tra=tr0;
          Vector3d trb=tr0;
          tra[i] -= delta;
          trb[i] += delta;
          ta = SE3Quat(q0, tra); 
          tb = SE3Quat(q0, trb); 
        } else {
          Quaterniond qa=q0;
          Quaterniond qb=q0;
          if (i == 3) {
            qa.x() -= delta;
            qb.x() += delta;
          }
          else if (i == 4) {
            qa.y() -= delta;
            qb.y() += delta;
          }
          else if (i == 5) {
            qa.z() -= delta;
            qb.z() += delta;
          }
          qa.normalize();
          qb.normalize();
          ta = SE3Quat(qa, tr0); 
          tb = SE3Quat(qb, tr0); 
        }

        Vector3d dtr = (tb.translation() - ta.translation())*idelta;
        Vector3d taAngles, tbAngles;
        quat_to_euler(ta.rotation(), taAngles(2), taAngles(1), taAngles(0));
        quat_to_euler(tb.rotation(), tbAngles(2), tbAngles(1), tbAngles(0));
        Vector3d da = (tbAngles - taAngles) * idelta; //TODO wraparounds not handled

        for (int j=0; j<6; j++){
          if (j<3){
            J(j, i) = dtr(j);
          } else {
            J(j, i) = da(j-3);
          }
        }
      }
    }
  }

G2oSlamInterface::G2oSlamInterface(SparseOptimizerOnline* optimizer) :
  _optimizer(optimizer), _firstOptimization(true), _nodesAdded(0),
  _incIterations(1), _updateGraphEachN(10), _batchEveryN(100),
  _lastBatchStep(0), _initSolverDone(false)
{
}

bool G2oSlamInterface::addNode(const std::string& tag, int id, int dimension, const std::vector<double>& values)
{
  // allocating the desired solver + testing whether the solver is okay
  if (! _initSolverDone) {
    _initSolverDone = true;
    _optimizer->initSolver(dimension, _batchEveryN);
  }

  // we add the node when we are asked to add the according edge
  (void) tag;
  (void) id;
  (void) dimension;
  (void) values;
  return true;
}

bool G2oSlamInterface::addEdge(const std::string& tag, int id, int dimension, int v1Id, int v2Id, const std::vector<double>& measurement, const std::vector<double>& information)
{
  (void) tag;
  (void) id;
  size_t oldEdgesSize = _optimizer->edges().size();

  if (dimension == 3) {

    SE2 transf(measurement[0], measurement[1], measurement[2]);
    Eigen::Matrix3d infMat;
    int idx = 0;
    for (int r = 0; r < 3; ++r)
      for (int c = r; c < 3; ++c, ++idx) {
        assert(idx < (int)information.size());
        infMat(r,c) = infMat(c,r) = information[idx];
      }
    //cerr << PVAR(infMat) << endl;

    int doInit = 0;
    SparseOptimizer::Vertex* v1 = _optimizer->vertex(v1Id);
    SparseOptimizer::Vertex* v2 = _optimizer->vertex(v2Id);
    if (! v1) {
      OptimizableGraph::Vertex* v = v1 = addVertex(dimension, v1Id);
      _verticesAdded.insert(v);
      doInit = 1;
      ++_nodesAdded;
    }

    if (! v2) {
      OptimizableGraph::Vertex* v = v2 = addVertex(dimension, v2Id);
      _verticesAdded.insert(v);
      doInit = 2;
      ++_nodesAdded;
    }

    if (_optimizer->edges().size() == 0) {
      cerr << "FIRST EDGE ";
      if (v1->id() < v2->id()) {
        cerr << "fixing " << v1->id() << endl;
        v1->setFixed(true);
      }
      else {
        cerr << "fixing " << v2->id() << endl;
        v2->setFixed(true);
      }
    }

    OnlineEdgeSE2* e = new OnlineEdgeSE2;
    e->vertices()[0] = v1;
    e->vertices()[1] = v2;
    e->setMeasurement(transf);
    e->setInformation(infMat);
    _optimizer->addEdge(e);
    _edgesAdded.insert(e);

    if (doInit) {
      OptimizableGraph::Vertex* from = static_cast<OptimizableGraph::Vertex*>(e->vertices()[0]);
      OptimizableGraph::Vertex* to   = static_cast<OptimizableGraph::Vertex*>(e->vertices()[1]);
      switch (doInit){
        case 1: // initialize v1 from v2
          {
            HyperGraph::VertexSet toSet;
            toSet.insert(to);
            if (e->initialEstimatePossible(toSet, from) > 0.) {
              e->initialEstimate(toSet, from);
            }
            break;
          }
        case 2: 
          {
            HyperGraph::VertexSet fromSet;
            fromSet.insert(from);
            if (e->initialEstimatePossible(fromSet, to) > 0.) {
              e->initialEstimate(fromSet, to);  
            }
            break;
          }
        default: cerr << "doInit wrong value\n"; 
      }
    }

  }
  else if (dimension == 6) {

    Eigen::Isometry3d transf;
    Matrix<double, 6, 6> infMat;

    if (measurement.size() == 7) { // measurement is a Quaternion
      Vector7d meas;
      for (int i=0; i<7; ++i) 
        meas(i) = measurement[i];
      // normalize the quaternion to recover numerical precision lost by storing as human readable text
      Vector4d::MapType(meas.data()+3).normalize();
      transf = internal::fromVectorQT(meas);

      for (int i = 0, idx = 0; i < infMat.rows(); ++i)
        for (int j = i; j < infMat.cols(); ++j){
          infMat(i,j) = information[idx++];
          if (i != j)
            infMat(j,i)=infMat(i,j);
        }
    } else { // measurement consists of Euler angles
      Vector6d aux;
      aux << measurement[0], measurement[1], measurement[2],measurement[3], measurement[4], measurement[5];
      transf = internal::fromVectorET(aux);
      Matrix<double, 6, 6> infMatEuler;
      int idx = 0;
      for (int r = 0; r < 6; ++r)
        for (int c = r; c < 6; ++c, ++idx) {
          assert(idx < (int)information.size());
          infMatEuler(r,c) = infMatEuler(c,r) = information[idx];
        }
      // convert information matrix to our internal representation
      Matrix<double, 6, 6> J;
      SE3Quat transfAsSe3(transf.matrix().topLeftCorner<3,3>(), transf.translation());
      jac_quat3_euler3(J, transfAsSe3);
      infMat.noalias() = J.transpose() * infMatEuler * J;
      //cerr << PVAR(transf.matrix()) << endl;
      //cerr << PVAR(infMat) << endl;
    }

    int doInit = 0;
    SparseOptimizer::Vertex* v1 = _optimizer->vertex(v1Id);
    SparseOptimizer::Vertex* v2 = _optimizer->vertex(v2Id);
    if (! v1) {
      OptimizableGraph::Vertex* v = v1 = addVertex(dimension, v1Id);
      _verticesAdded.insert(v);
      doInit = 1;
      ++_nodesAdded;
    }

    if (! v2) {
      OptimizableGraph::Vertex* v = v2 = addVertex(dimension, v2Id);
      _verticesAdded.insert(v);
      doInit = 2;
      ++_nodesAdded;
    }

    if (_optimizer->edges().size() == 0) {
      cerr << "FIRST EDGE ";
      if (v1->id() < v2->id()) {
        cerr << "fixing " << v1->id() << endl;
        v1->setFixed(true);
      }
      else {
        cerr << "fixing " << v2->id() << endl;
        v2->setFixed(true);
      }
    }

    OnlineEdgeSE3* e = new OnlineEdgeSE3;
    e->vertices()[0] = v1;
    e->vertices()[1] = v2;
    e->setMeasurement(transf);
    e->setInformation(infMat);
    _optimizer->addEdge(e);
    _edgesAdded.insert(e);

    if (doInit) {
      OptimizableGraph::Vertex* from = static_cast<OptimizableGraph::Vertex*>(e->vertices()[0]);
      OptimizableGraph::Vertex* to   = static_cast<OptimizableGraph::Vertex*>(e->vertices()[1]);
      switch (doInit){
        case 1: // initialize v1 from v2
          {
            HyperGraph::VertexSet toSet;
            toSet.insert(to);
            if (e->initialEstimatePossible(toSet, from) > 0.) {
              e->initialEstimate(toSet, from);
            }
            break;
          }
        case 2: 
          {
            HyperGraph::VertexSet fromSet;
            fromSet.insert(from);
            if (e->initialEstimatePossible(fromSet, to) > 0.) {
              e->initialEstimate(fromSet, to);  
            }
            break;
          }
        default: cerr << "doInit wrong value\n"; 
      }
    }

  }
  else {
    cerr << __PRETTY_FUNCTION__ << " not implemented for this dimension" << endl;
    return false;
  }

  if (oldEdgesSize == 0) {
    _optimizer->jacobianWorkspace().allocate();
  }

  return true;
}

bool G2oSlamInterface::fixNode(const std::vector<int>& nodes)
{
  for (size_t i = 0; i < nodes.size(); ++i) {
    OptimizableGraph::Vertex* v = _optimizer->vertex(nodes[i]);
    if (v)
      v->setFixed(true);
  }
  return true;
}

bool G2oSlamInterface::queryState(const std::vector<int>& nodes)
{
  //return true;
  cout << "BEGIN" << endl;
#if 1
  if (nodes.size() == 0) {
    for (OptimizableGraph::VertexIDMap::const_iterator it = _optimizer->vertices().begin(); it != _optimizer->vertices().end(); ++it) {
      OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(it->second);
      printVertex(v);
    }
  } else {
    for (size_t i = 0; i < nodes.size(); ++i) {
      OptimizableGraph::Vertex* v = _optimizer->vertex(nodes[i]);
      if (v)
        printVertex(v);
    }
  }
#endif
  cout << "END" << endl << flush;

  return true;
}

bool G2oSlamInterface::solveState()
{
  SolveResult state = solve();
  return state != ERROR;
}

OptimizableGraph::Vertex* G2oSlamInterface::addVertex(int dimension, int id)
{
  if (dimension == 3) {
    OnlineVertexSE2* v =  new OnlineVertexSE2;
    v->setId(id); // estimate will be set later when the edge is added
    _optimizer->addVertex(v);
    return v;
  }
  else if (dimension == 6) {
    OnlineVertexSE3* v =  new OnlineVertexSE3;
    v->setId(id); // estimate will be set later when the edge is added
    _optimizer->addVertex(v);
    return v;
  }
  else {
    return 0;
  }
}

bool G2oSlamInterface::printVertex(OptimizableGraph::Vertex* v)
{
  static char buffer[10000]; // that should be more than enough
  int vdim = v->dimension();
  if (vdim == 3) {
    char* s = buffer;
    OnlineVertexSE2* v2 = static_cast<OnlineVertexSE2*>(v);
    memcpy(s, "VERTEX_XYT ", 11);
    s += 11;
    s += modp_itoa10(v->id(), s);
    *s++ = ' ';
    s += modp_dtoa(v2->updatedEstimate.translation().x(), s, 6);
    *s++ = ' ';
    s += modp_dtoa(v2->updatedEstimate.translation().y(), s, 6);
    *s++ = ' ';
    s += modp_dtoa(v2->updatedEstimate.rotation().angle(), s, 6);
    *s++ = '\n';
    cout.write(buffer, s - buffer);
    return true;
  }
  else if (vdim == 6) {
    char* s = buffer;
    OnlineVertexSE3* v3 = static_cast<OnlineVertexSE3*>(v);
    Vector3d eulerAngles = internal::toEuler(v3->updatedEstimate.matrix().topLeftCorner<3,3>());
    const double& roll = eulerAngles(0);
    const double& pitch = eulerAngles(1);
    const double& yaw = eulerAngles(2);
    memcpy(s, "VERTEX_XYZRPY ", 14);
    s += 14;
    s += modp_itoa10(v->id(), s);
    *s++ = ' ';
    s += modp_dtoa(v3->updatedEstimate.translation().x(), s, 6);
    *s++ = ' ';
    s += modp_dtoa(v3->updatedEstimate.translation().y(), s, 6);
    *s++ = ' ';
    s += modp_dtoa(v3->updatedEstimate.translation().z(), s, 6);
    *s++ = ' ';
    s += modp_dtoa(roll, s, 6);
    *s++ = ' ';
    s += modp_dtoa(pitch, s, 6);
    *s++ = ' ';
    s += modp_dtoa(yaw, s, 6);
    *s++ = '\n';
    cout.write(buffer, s - buffer);
    return true;
  }
  return false;
}

void G2oSlamInterface::setUpdateGraphEachN(int n)
{
  _updateGraphEachN = n;
}

G2oSlamInterface::SolveResult G2oSlamInterface::solve()
{
  if (_nodesAdded >= _updateGraphEachN) {

    // decide on batch step or normal step
    _optimizer->batchStep = false;
    if ((int)_optimizer->vertices().size() - _lastBatchStep >= _batchEveryN) {
      _lastBatchStep = _optimizer->vertices().size();
      _optimizer->batchStep = true;
    }

    if (_firstOptimization) {
      if (!_optimizer->initializeOptimization()){
        cerr << "initialization failed" << endl;
        return ERROR;
      }
    } else {
      if (! _optimizer->updateInitialization(_verticesAdded, _edgesAdded)) {
        cerr << "updating initialization failed" << endl;
        return ERROR;
      }
    }

    int currentIt = _optimizer->optimize(_incIterations, !_firstOptimization); (void) currentIt;
    _firstOptimization = false;
    _nodesAdded = 0;
    _verticesAdded.clear();
    _edgesAdded.clear();
    if (_optimizer->batchStep)
      return SOLVED_BATCH;
    return SOLVED;
  }

  return NOOP;
}

void G2oSlamInterface::setBatchSolveEachN(int n)
{
  _batchEveryN = n;
}

} // end namespace
