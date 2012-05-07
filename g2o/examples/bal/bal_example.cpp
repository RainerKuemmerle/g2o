// g2o - General Graph Optimization
// Copyright (C) 2012 R. Kümmerle
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
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>

#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"

#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"

using namespace g2o;
using namespace Eigen;
using namespace std;

class CameraBAL : public SE3Quat
{
  public:
    CameraBAL():
      SE3Quat(),
      focalLength(1.), radial1(1.), radial2(2.)
    {
    }

    CameraBAL(const Quaterniond& q, const Vector3d& t, double focalLength_, double radial1_, double radial2_):
      SE3Quat(q, t),
      focalLength(focalLength_), radial1(radial1_), radial2(radial2_)
    {
    }

    double focalLength;
    double radial1;
    double radial2;
};

class VertexCameraBAL : public BaseVertex<9, CameraBAL>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCameraBAL()
    {
    }

    virtual bool read(std::istream& /*is*/)
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }

    virtual void setToOriginImpl()
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
    }

    virtual void oplusImpl(const double* update)
    {
      Vector6d::ConstMapType v(update);
      SE3Quat increment(v);
      _estimate *= increment;

      _estimate.focalLength += update[6];
      _estimate.radial1 += update[7];
      _estimate.radial2 += update[8];
    }
};

class EdgeObservationBAL : public BaseBinaryEdge<2, Vector2d, VertexCameraBAL, VertexPointXYZ>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeObservationBAL()
    {
    }
    virtual bool read(std::istream& /*is*/)
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }
    virtual bool write(std::ostream& /*os*/) const
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }

    void computeError()
    {
      const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*>(vertex(0));
      const VertexPointXYZ* point = static_cast<const VertexPointXYZ*>(vertex(1));

      // conversion from world to camera coordinates
      Eigen::Vector3d p = cam->estimate() * point->estimate();

      // perspective division
      Eigen::Vector2d projectedPoint;
      projectedPoint(0) = - p(0) / p(2);
      projectedPoint(1) = - p(1) / p(2);

      // conversion to pixel coordinates
      double radiusSqr = projectedPoint.squaredNorm();
      double r_p = 1. + cam->estimate().radial1 * radiusSqr + cam->estimate().radial2 * radiusSqr * radiusSqr;
      Eigen::Vector2d prediction;
      prediction(0) = cam->estimate().focalLength * r_p * projectedPoint(0);
      prediction(1) = cam->estimate().focalLength * r_p * projectedPoint(1);

      _error = prediction - measurement();
      //cout << PVAR(prediction.transpose()) << "\t\t" << PVAR(measurement().transpose()) << endl;
    }
};

int main(int argc, char** argv)
{
  int maxIterations;
  bool verbose;
  string inputFilename;
  std::vector<int> gaugeList;
  CommandArgs arg;
  arg.param("i", maxIterations, 5, "perform n iterations");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.paramLeftOver("graph-input", inputFilename, "", "file which will be processed");

  arg.parseArgs(argc, argv);

  typedef g2o::BlockSolver< g2o::BlockSolverTraits<9, 3> >  BalBlockSolver;
  typedef g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType> BalLinearSolver;

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  BalLinearSolver * linearSolver = new BalLinearSolver();
  linearSolver->setBlockOrdering(true);
  BalBlockSolver* solver_ptr = new BalBlockSolver(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  solver->setUserLambdaInit(1000.);
  optimizer.setAlgorithm(solver);

  vector<VertexPointXYZ*> points;
  vector<VertexCameraBAL*> cameras;

  // parse BAL dataset
  {
    ifstream ifs(inputFilename.c_str());
    int numCameras, numPoints, numObservations;
    ifs >> numCameras >> numPoints >> numObservations;

    cerr << PVAR(numCameras) << " " << PVAR(numPoints) << " " << PVAR(numObservations) << endl;

    int id = 0;
    cameras.reserve(numCameras);
    for (int i = 0; i < numCameras; ++i, ++id) {
      VertexCameraBAL* cam = new VertexCameraBAL;
      cam->setId(id);
      cam->setFixed(id == 0);
      optimizer.addVertex(cam);
      cameras.push_back(cam);
    }

    points.reserve(numPoints);
    for (int i = 0; i < numPoints; ++i, ++id) {
      VertexPointXYZ* p = new VertexPointXYZ;
      p->setId(id);
      p->setMarginalized(true);
      bool addedVertex = optimizer.addVertex(p);
      if (! addedVertex) {
        cerr << "failing adding vertex" << endl;
      }
      points.push_back(p);
    }

    // read in the observation
    for (int i = 0; i < numObservations; ++i) {
      int camIndex, pointIndex;
      double obsX, obsY;
      ifs >> camIndex >> pointIndex >> obsX >> obsY;

      assert(camIndex >= 0 && (size_t)camIndex < cameras.size() && "Index out of bounds");
      VertexCameraBAL* cam = cameras[camIndex];
      assert(pointIndex >= 0 && (size_t)pointIndex < points.size() && "Index out of bounds");
      VertexPointXYZ* point = points[pointIndex];

      EdgeObservationBAL* e = new EdgeObservationBAL;
      e->setVertex(0, cam);
      e->setVertex(1, point);
      e->setInformation(Eigen::Matrix2d::Identity());
      e->setMeasurement(Eigen::Vector2d(obsX, obsY));
      bool addedEdge = optimizer.addEdge(e);
      if (! addedEdge) {
        cerr << "error adding edge" << endl;
      }
    }

    // read in the camera params
    for (int i = 0; i < numCameras; ++i) {
      Eigen::Vector3d rotation;
      Eigen::Vector3d translation;
      double f, k1, k2;
      ifs >> rotation(0) >> rotation(1) >> rotation(2);
      ifs >> translation(0) >> translation(1) >> translation(2);
      ifs >> f >> k1 >> k2;
      Eigen::AngleAxisd rot(rotation.norm(), rotation.normalized());

      VertexCameraBAL* cam = cameras[i];
      cam->setEstimate(CameraBAL((Eigen::Quaterniond)rot, translation, f, k1, k2));
    }

    // read in the points
    for (int i = 0; i < numPoints; ++i) {
      Eigen::Vector3d p;
      ifs >> p(0) >> p(1) >> p(2);

      VertexPointXYZ* point = points[i];
      point->setEstimate(p);
    }

  }

  
  optimizer.initializeOptimization();
  optimizer.setVerbose(verbose);
  optimizer.optimize(maxIterations);

  // dump the points
  ofstream fout("points.wrl"); // loadable with meshlab
  fout 
    << "#VRML V2.0 utf8\n"
    << "Shape {\n"
    << "  appearance Appearance {\n"
    << "    material Material {\n"
    << "      diffuseColor " << 1 << " " << 0 << " " << 0 << "\n"
    << "      ambientIntensity 0.2\n"
    << "      emissiveColor 0.0 0.0 0.0\n"
    << "      specularColor 0.0 0.0 0.0\n"
    << "      shininess 0.2\n"
    << "      transparency 0.0\n"
    << "    }\n"
    << "  }\n"
    << "  geometry PointSet {\n"
    << "    coord Coordinate {\n"
    << "      point [\n";
  for (vector<VertexPointXYZ*>::const_iterator it = points.begin(); it != points.end(); ++it) {
    fout << (*it)->estimate().transpose() << endl;
  }
  fout << "    ]\n" << "  }\n" << "}\n" << "  }\n";
  
  return 0;
}
