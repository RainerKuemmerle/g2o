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
#include "g2o/core/batch_stats.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"

#if defined G2O_HAVE_CHOLMOD
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#elif defined G2O_HAVE_CSPARSE
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#endif

using namespace g2o;
using namespace std;

/**
 * \brief camera vertex which stores the parameters for a pinhole camera
 *
 * The parameters of the camera are 
 * - rx,ry,rz representing the rotation axis, whereas the angle is given by ||(rx,ry,rz)||
 * - tx,ty,tz the translation of the camera
 * - f the focal length of the camera
 * - k1, k2 two radial distortion parameters
 */
class VertexCameraBAL : public BaseVertex<9, Eigen::VectorXd>
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
      Eigen::VectorXd::ConstMapType v(update, 9);
      _estimate += v;
    }
};

/**
 * \brief 3D world feature
 *
 * A 3D point feature in the world
 */
class VertexPointBAL : public BaseVertex<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPointBAL()
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
      Eigen::Vector3d::ConstMapType v(update);
      _estimate += v;
    }
};

/**
 * \brief edge representing the observation of a world feature by a camera
 *
 * see: http://grail.cs.washington.edu/projects/bal/
 * We use a pinhole camera model; the parameters we estimate for each camera
 * area rotation R, a translation t, a focal length f and two radial distortion
 * parameters k1 and k2. The formula for projecting a 3D point X into a camera
 * R,t,f,k1,k2 is:
 * P  =  R * X + t       (conversion from world to camera coordinates)
 * p  = -P / P.z         (perspective division)
 * p' =  f * r(p) * p    (conversion to pixel coordinates) where P.z is the third (z) coordinate of P.
 *
 * In the last equation, r(p) is a function that computes a scaling factor to undo the radial
 * distortion:
 * r(p) = 1.0 + k1 * ||p||^2 + k2 * ||p||^4. 
 *
 * This gives a projection in pixels, where the origin of the image is the
 * center of the image, the positive x-axis points right, and the positive
 * y-axis points up (in addition, in the camera coordinate system, the positive
 * z-axis points backwards, so the camera is looking down the negative z-axis,
 * as in OpenGL).
 */
class EdgeObservationBAL : public BaseBinaryEdge<2, Vector2d, VertexCameraBAL, VertexPointBAL>
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
      const VertexPointBAL* point = static_cast<const VertexPointBAL*>(vertex(1));

      // conversion from world to camera coordinates
      Eigen::Vector3d::ConstMapType angleAxisVector(cam->estimate().data(), 3);

      // Rodrigues' formula for the rotation
      Eigen::Vector3d p; 
      double theta = angleAxisVector.norm();
      if (theta > 0.) {
        Eigen::Vector3d v = angleAxisVector / theta;
        double cth = cos(theta);
        double sth = sin(theta);
        p = point->estimate() * cth + v.cross(point->estimate()) * sth + v * v.dot(point->estimate()) * (1. - cth);
      } else {
        p = point->estimate();
      }

      // translation of the camera
      Eigen::Vector3d::ConstMapType translation(cam->estimate().data()+3, 3);
      p += translation;

      // perspective division
      Eigen::Vector2d projectedPoint;
      projectedPoint(0) = - p(0) / p(2);
      projectedPoint(1) = - p(1) / p(2);

      // conversion to pixel coordinates
      double radiusSqr = projectedPoint.squaredNorm();
      const double& f  = cam->estimate()(6);
      const double& k1 = cam->estimate()(7);
      const double& k2 = cam->estimate()(8);
      double r_p       = 1. + k1 * radiusSqr + k2 * radiusSqr * radiusSqr;
      Eigen::Vector2d prediction;
      prediction(0) = f * r_p * projectedPoint(0);
      prediction(1) = f * r_p * projectedPoint(1);

      _error = prediction - measurement();
      //cout << PVAR(prediction.transpose()) << "\t\t" << PVAR(measurement().transpose()) << endl;
    }
};

int main(int argc, char** argv)
{
  int maxIterations;
  bool verbose;
  string inputFilename;
  string statsFilename;
  CommandArgs arg;
  arg.param("i", maxIterations, 5, "perform n iterations");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("stats", statsFilename, "", "specify a file for the statistics");
  arg.paramLeftOver("graph-input", inputFilename, "", "file which will be processed");

  arg.parseArgs(argc, argv);

  typedef g2o::BlockSolver< g2o::BlockSolverTraits<9, 3> >  BalBlockSolver;
#ifdef G2O_HAVE_CHOLMOD
  typedef g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType> BalLinearSolver;
#elif defined G2O_HAVE_CSPARSE
  typedef g2o::LinearSolverCSparse<BalBlockSolver::PoseMatrixType> BalLinearSolver;
#else
#error neither CSparse nor Cholmod are available
#endif

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  BalLinearSolver * linearSolver = new BalLinearSolver();
  linearSolver->setBlockOrdering(true);
  BalBlockSolver* solver_ptr = new BalBlockSolver(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  solver->setUserLambdaInit(1);
  optimizer.setAlgorithm(solver);
  if (statsFilename.size() > 0){
    optimizer.setComputeBatchStatistics(true);
  }

  vector<VertexPointBAL*> points;
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
      //cam->setFixed(id == 0);
      optimizer.addVertex(cam);
      cameras.push_back(cam);
    }

    points.reserve(numPoints);
    for (int i = 0; i < numPoints; ++i, ++id) {
      VertexPointBAL* p = new VertexPointBAL;
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
      VertexPointBAL* point = points[pointIndex];

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
    Eigen::VectorXd cameraParameter(9);
    for (int i = 0; i < numCameras; ++i) {
      for (int j = 0; j < 9; ++j)
        ifs >> cameraParameter(j);
      VertexCameraBAL* cam = cameras[i];
      cam->setEstimate(cameraParameter);
    }

    // read in the points
    Eigen::Vector3d p;
    for (int i = 0; i < numPoints; ++i) {
      ifs >> p(0) >> p(1) >> p(2);

      VertexPointBAL* point = points[i];
      point->setEstimate(p);
    }

  }

  optimizer.initializeOptimization();
  optimizer.setVerbose(verbose);
  optimizer.optimize(maxIterations);

  if (statsFilename!=""){
    cerr << "writing stats to file \"" << statsFilename << "\" ... ";
    ofstream fout(statsFilename.c_str());
    const BatchStatisticsContainer& bsc = optimizer.batchStatistics();
    for (size_t i=0; i<bsc.size(); i++)
      fout << bsc[i] << endl;
    cerr << "done." << endl;
  }

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
  for (vector<VertexPointBAL*>::const_iterator it = points.begin(); it != points.end(); ++it) {
    fout << (*it)->estimate().transpose() << endl;
  }
  fout << "    ]\n" << "  }\n" << "}\n" << "  }\n";
  
  return 0;
}
