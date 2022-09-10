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

#include <iostream>

#include "fast_output.h"
#include "g2o/types/slam3d/se3quat.h"
#include "graph_optimizer_sparse_online.h"
#include "types_slam2d_online.h"
#include "types_slam3d_online.h"

using std::cerr;
using std::cout;
using std::endl;
using std::flush;

namespace g2o {

namespace {
void quat_to_euler(const Quaternion& q, double& yaw, double& pitch,
                   double& roll) {
  const double& q0 = q.w();
  const double& q1 = q.x();
  const double& q2 = q.y();
  const double& q3 = q.z();
  roll = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
  pitch = asin(2 * (q0 * q2 - q3 * q1));
  yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
}

void jac_quat3_euler3(MatrixN<6>& J, const SE3Quat& t) {
  const Vector3& tr0 = t.translation();
  const Quaternion& q0 = t.rotation();

  double delta = 1e-6;
  double idelta = 1. / (2. * delta);

  for (int i = 0; i < 6; i++) {
    SE3Quat ta;
    SE3Quat tb;
    if (i < 3) {
      Vector3 tra = tr0;
      Vector3 trb = tr0;
      tra[i] -= delta;
      trb[i] += delta;
      ta = SE3Quat(q0, tra);
      tb = SE3Quat(q0, trb);
    } else {
      Quaternion qa = q0;
      Quaternion qb = q0;
      if (i == 3) {
        qa.x() -= delta;
        qb.x() += delta;
      } else if (i == 4) {
        qa.y() -= delta;
        qb.y() += delta;
      } else if (i == 5) {
        qa.z() -= delta;
        qb.z() += delta;
      }
      qa.normalize();
      qb.normalize();
      ta = SE3Quat(qa, tr0);
      tb = SE3Quat(qb, tr0);
    }

    Vector3 dtr = (tb.translation() - ta.translation()) * idelta;
    Vector3 taAngles;
    Vector3 tbAngles;
    quat_to_euler(ta.rotation(), taAngles(2), taAngles(1), taAngles(0));
    quat_to_euler(tb.rotation(), tbAngles(2), tbAngles(1), tbAngles(0));
    Vector3 da =
        (tbAngles - taAngles) * idelta;  // TODO(goki): wraparounds not handled

    for (int j = 0; j < 6; j++) {
      if (j < 3) {
        J(j, i) = dtr(j);
      } else {
        J(j, i) = da(j - 3);
      }
    }
  }
}
}  // namespace

G2oSlamInterface::G2oSlamInterface(SparseOptimizerOnline* optimizer)
    : optimizer_(optimizer) {}

bool G2oSlamInterface::addNode(const std::string& tag, int id, int dimension,
                               const std::vector<double>& values) {
  // allocating the desired solver + testing whether the solver is okay
  if (!initSolverDone_) {
    initSolverDone_ = true;
    optimizer_->initSolver(dimension, batchEveryN_);
  }

  // we add the node when we are asked to add the according edge
  (void)tag;
  (void)id;
  (void)dimension;
  (void)values;
  return true;
}

bool G2oSlamInterface::addEdge(const std::string& tag, int id, int dimension,
                               int v1Id, int v2Id,
                               const std::vector<double>& measurement,
                               const std::vector<double>& information) {
  (void)tag;
  (void)id;
  size_t oldEdgesSize = optimizer_->edges().size();

  if (dimension == 3) {
    SE2 transf(measurement[0], measurement[1], measurement[2]);
    Eigen::Matrix3d infMat;
    int idx = 0;
    for (int r = 0; r < 3; ++r)
      for (int c = r; c < 3; ++c, ++idx) {
        assert(idx < (int)information.size());
        infMat(r, c) = infMat(c, r) = information[idx];
      }
    // cerr << PVAR(infMat) << endl;

    int doInit = 0;
    auto v1 = optimizer_->vertex(v1Id);
    auto v2 = optimizer_->vertex(v2Id);
    if (!v1) {
      auto v = v1 = addVertex(dimension, v1Id);
      verticesAdded_.insert(v);
      doInit = 1;
      ++nodesAdded_;
    }

    if (!v2) {
      auto v = v2 = addVertex(dimension, v2Id);
      verticesAdded_.insert(v);
      doInit = 2;
      ++nodesAdded_;
    }

    if (optimizer_->edges().empty()) {
      cerr << "FIRST EDGE ";
      if (v1->id() < v2->id()) {
        cerr << "fixing " << v1->id() << endl;
        v1->setFixed(true);
      } else {
        cerr << "fixing " << v2->id() << endl;
        v2->setFixed(true);
      }
    }

    auto e = std::make_shared<OnlineEdgeSE2>();
    e->vertices()[0] = v1;
    e->vertices()[1] = v2;
    e->setMeasurement(transf);
    e->setInformation(infMat);
    optimizer_->addEdge(e);
    edgesAdded_.insert(e);

    if (doInit) {
      auto from =
          std::static_pointer_cast<OptimizableGraph::Vertex>(e->vertices()[0]);
      auto to =
          std::static_pointer_cast<OptimizableGraph::Vertex>(e->vertices()[1]);
      switch (doInit) {
        case 1:  // initialize v1 from v2
        {
          HyperGraph::VertexSet toSet;
          toSet.insert(to);
          if (e->initialEstimatePossible(toSet, from.get()) > 0.) {
            e->initialEstimate(toSet, from.get());
          }
          break;
        }
        case 2: {
          HyperGraph::VertexSet fromSet;
          fromSet.insert(from);
          if (e->initialEstimatePossible(fromSet, to.get()) > 0.) {
            e->initialEstimate(fromSet, to.get());
          }
          break;
        }
        default:
          cerr << "doInit wrong value\n";
      }
    }

  } else if (dimension == 6) {
    Eigen::Isometry3d transf;
    MatrixN<6> infMat;

    if (measurement.size() == 7) {  // measurement is a Quaternion
      Vector7 meas;
      for (int i = 0; i < 7; ++i) meas(i) = measurement[i];
      // normalize the quaternion to recover numerical precision lost by storing
      // as human readable text
      Vector4::MapType(meas.data() + 3).normalize();
      transf = internal::fromVectorQT(meas);

      for (int i = 0, idx = 0; i < infMat.rows(); ++i)
        for (int j = i; j < infMat.cols(); ++j) {
          infMat(i, j) = information[idx++];
          if (i != j) infMat(j, i) = infMat(i, j);
        }
    } else {  // measurement consists of Euler angles
      Vector6 aux;
      aux << measurement[0], measurement[1], measurement[2], measurement[3],
          measurement[4], measurement[5];
      transf = internal::fromVectorET(aux);
      MatrixN<6> infMatEuler;
      int idx = 0;
      for (int r = 0; r < 6; ++r)
        for (int c = r; c < 6; ++c, ++idx) {
          assert(idx < (int)information.size());
          infMatEuler(r, c) = infMatEuler(c, r) = information[idx];
        }
      // convert information matrix to our internal representation
      MatrixN<6> J;
      SE3Quat transfAsSe3(transf.matrix().topLeftCorner<3, 3>(),
                          transf.translation());
      jac_quat3_euler3(J, transfAsSe3);
      infMat.noalias() = J.transpose() * infMatEuler * J;
      // cerr << PVAR(transf.matrix()) << endl;
      // cerr << PVAR(infMat) << endl;
    }

    int doInit = 0;
    auto v1 = optimizer_->vertex(v1Id);
    auto v2 = optimizer_->vertex(v2Id);
    if (!v1) {
      auto v = v1 = addVertex(dimension, v1Id);
      verticesAdded_.insert(v);
      doInit = 1;
      ++nodesAdded_;
    }

    if (!v2) {
      auto v = v2 = addVertex(dimension, v2Id);
      verticesAdded_.insert(v);
      doInit = 2;
      ++nodesAdded_;
    }

    if (optimizer_->edges().empty()) {
      cerr << "FIRST EDGE ";
      if (v1->id() < v2->id()) {
        cerr << "fixing " << v1->id() << endl;
        v1->setFixed(true);
      } else {
        cerr << "fixing " << v2->id() << endl;
        v2->setFixed(true);
      }
    }

    auto e = std::make_shared<OnlineEdgeSE3>();
    e->vertices()[0] = v1;
    e->vertices()[1] = v2;
    e->setMeasurement(transf);
    e->setInformation(infMat);
    optimizer_->addEdge(e);
    edgesAdded_.insert(e);

    if (doInit) {
      auto from =
          std::static_pointer_cast<OptimizableGraph::Vertex>(e->vertices()[0]);
      auto to =
          std::static_pointer_cast<OptimizableGraph::Vertex>(e->vertices()[1]);
      switch (doInit) {
        case 1:  // initialize v1 from v2
        {
          HyperGraph::VertexSet toSet;
          toSet.insert(to);
          if (e->initialEstimatePossible(toSet, from.get()) > 0.) {
            e->initialEstimate(toSet, from.get());
          }
          break;
        }
        case 2: {
          HyperGraph::VertexSet fromSet;
          fromSet.insert(from);
          if (e->initialEstimatePossible(fromSet, to.get()) > 0.) {
            e->initialEstimate(fromSet, to.get());
          }
          break;
        }
        default:
          cerr << "doInit wrong value\n";
      }
    }

  } else {
    cerr << __PRETTY_FUNCTION__ << " not implemented for this dimension"
         << endl;
    return false;
  }

  if (oldEdgesSize == 0) {
    optimizer_->jacobianWorkspace().allocate();
  }

  return true;
}

bool G2oSlamInterface::fixNode(const std::vector<int>& nodes) {
  for (int node : nodes) {
    OptimizableGraph::Vertex* v = optimizer_->vertex(node).get();
    if (v) v->setFixed(true);
  }
  return true;
}

bool G2oSlamInterface::queryState(const std::vector<int>& nodes) {
  // return true;
  cout << "BEGIN" << endl;
#if 1
  if (nodes.empty()) {
    for (const auto& it : optimizer_->vertices()) {
      auto* v = static_cast<OptimizableGraph::Vertex*>(it.second.get());
      printVertex(v);
    }
  } else {
    for (int node : nodes) {
      OptimizableGraph::Vertex* v = optimizer_->vertex(node).get();
      if (v) printVertex(v);
    }
  }
#endif
  cout << "END" << endl << flush;

  return true;
}

bool G2oSlamInterface::solveState() {
  SolveResult state = solve();
  return state != kError;
}

std::shared_ptr<OptimizableGraph::Vertex> G2oSlamInterface::addVertex(
    int dimension, int id) {
  if (dimension == 3) {
    auto v = std::make_shared<OnlineVertexSE2>();
    v->setId(id);  // estimate will be set later when the edge is added
    optimizer_->addVertex(v);
    return v;
  }
  if (dimension == 6) {
    auto v = std::make_shared<OnlineVertexSE3>();
    v->setId(id);  // estimate will be set later when the edge is added
    optimizer_->addVertex(v);
    return v;
  }
  return nullptr;
}

bool G2oSlamInterface::printVertex(OptimizableGraph::Vertex* v) {
  static char buffer[10000];  // that should be more than enough
  int vdim = v->dimension();
  if (vdim == 3) {
    char* s = buffer;
    auto* v2 = static_cast<OnlineVertexSE2*>(v);
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
  if (vdim == 6) {
    char* s = buffer;
    auto* v3 = static_cast<OnlineVertexSE3*>(v);
    Vector3 eulerAngles =
        internal::toEuler(v3->updatedEstimate.matrix().topLeftCorner<3, 3>());
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

void G2oSlamInterface::setUpdateGraphEachN(int n) { updateGraphEachN_ = n; }

G2oSlamInterface::SolveResult G2oSlamInterface::solve() {
  if (nodesAdded_ >= updateGraphEachN_) {
    // decide on batch step or normal step
    optimizer_->batchStep = false;
    if (static_cast<int>(optimizer_->vertices().size()) - lastBatchStep_ >=
        batchEveryN_) {
      lastBatchStep_ = optimizer_->vertices().size();
      optimizer_->batchStep = true;
    }

    if (firstOptimization_) {
      if (!optimizer_->initializeOptimization()) {
        cerr << "initialization failed" << endl;
        return kError;
      }
    } else {
      if (!optimizer_->updateInitialization(verticesAdded_, edgesAdded_)) {
        cerr << "updating initialization failed" << endl;
        return kError;
      }
    }

    int currentIt = optimizer_->optimize(incIterations_, !firstOptimization_);
    (void)currentIt;
    firstOptimization_ = false;
    nodesAdded_ = 0;
    verticesAdded_.clear();
    edgesAdded_.clear();
    if (optimizer_->batchStep) return kSolvedBatch;
    return kSolved;
  }

  return kNoop;
}

void G2oSlamInterface::setBatchSolveEachN(int n) { batchEveryN_ = n; }

}  // namespace g2o
