// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
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

#include <random>
#include <iostream>
#include <stdint.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/icp/types_icp.h"

using namespace Eigen;
using namespace std;
using namespace g2o;

// sampling distributions
  class Sample
  {

    static default_random_engine gen_real;
    static default_random_engine gen_int;
  public:
    static int uniform(int from, int to);

    static double uniform();

    static double gaussian(double sigma);
  };


  default_random_engine Sample::gen_real;
  default_random_engine Sample::gen_int;

  int Sample::uniform(int from, int to)
  {
    uniform_int_distribution<int> unif(from, to);
    int sam = unif(gen_int);
    return  sam;
  }

  double Sample::uniform()
  {
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    double sam = unif(gen_real);
    return  sam;
  }

  double Sample::gaussian(double sigma)
  {
    std::normal_distribution<double> gauss(0.0, sigma);
    double sam = gauss(gen_real);
    return  sam;
  }


//
// set up simulated system with noise, optimize it
//

int main(int argc, char **argv)
{
  int num_points = 0;

  // check for arg, # of points to use in projection SBA
  if (argc > 1)
    num_points = atoi(argv[1]);

  double euc_noise = 0.1;      // noise in position, m
  double pix_noise = 1.0;       // pixel noise
  //  double outlier_ratio = 0.1;


  SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  // variable-size block solver
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<BlockSolverX>(g2o::make_unique<LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>()));

  optimizer.setAlgorithm(solver);

  vector<Vector3d> true_points;
  for (size_t i=0;i<1000; ++i)
  {
    true_points.push_back(Vector3d((Sample::uniform()-0.5)*3,
                                   Sample::uniform()-0.5,
                                   Sample::uniform()+10));
  }


  // set up camera params
  Vector2d focal_length(500,500); // pixels
  Vector2d principal_point(320,240); // 640x480 image
  double baseline = 0.075;      // 7.5 cm baseline

  // set up camera params and projection matrices on vertices
  g2o::VertexSCam::setKcam(focal_length[0],focal_length[1],
                           principal_point[0],principal_point[1],
                           baseline);


  // set up two poses
  int vertex_id = 0;
  for (size_t i=0; i<2; ++i)
  {
    // set up rotation and translation for this node
    Vector3d t(0,0,i);
    Quaterniond q;
    q.setIdentity();

    Eigen::Isometry3d cam;           // camera pose
    cam = q;
    cam.translation() = t;

    // set up node
    VertexSCam *vc = new VertexSCam();
    vc->setEstimate(cam);
    vc->setId(vertex_id);      // vertex id

    cerr << t.transpose() << " | " << q.coeffs().transpose() << endl;

    // set first cam pose fixed
    if (i==0)
      vc->setFixed(true);

    // make sure projection matrices are set
    vc->setAll();

    // add to optimizer
    optimizer.addVertex(vc);

    vertex_id++;                
  }

  // set up point matches for GICP
  for (size_t i=0; i<true_points.size(); ++i)
  {
    // get two poses
    VertexSE3* vp0 = 
      dynamic_cast<VertexSE3*>(optimizer.vertices().find(0)->second);
    VertexSE3* vp1 = 
      dynamic_cast<VertexSE3*>(optimizer.vertices().find(1)->second);

    // calculate the relative 3D position of the point
    Vector3d pt0,pt1;
    pt0 = vp0->estimate().inverse() * true_points[i];
    pt1 = vp1->estimate().inverse() * true_points[i];

    // add in noise
    pt0 += Vector3d(Sample::gaussian(euc_noise ),
                    Sample::gaussian(euc_noise ),
                    Sample::gaussian(euc_noise ));

    pt1 += Vector3d(Sample::gaussian(euc_noise ),
                    Sample::gaussian(euc_noise ),
                    Sample::gaussian(euc_noise ));

    // form edge, with normals in varioius positions
    Vector3d nm0, nm1;
    nm0 << 0, i, 1;
    nm1 << 0, i, 1;
    nm0.normalize();
    nm1.normalize();

    Edge_V_V_GICP * e           // new edge with correct cohort for caching
        = new Edge_V_V_GICP(); 

    e->vertices()[0]            // first viewpoint
      = dynamic_cast<OptimizableGraph::Vertex*>(vp0);

    e->vertices()[1]            // second viewpoint
      = dynamic_cast<OptimizableGraph::Vertex*>(vp1);

    EdgeGICP meas;

    meas.pos0 = pt0;
    meas.pos1 = pt1;
    meas.normal0 = nm0;
    meas.normal1 = nm1;
    e->setMeasurement(meas);
    meas = e->measurement();
    //        e->inverseMeasurement().pos() = -kp;

    // use this for point-plane
    e->information() = meas.prec0(0.01);

    // use this for point-point 
    //    e->information().setIdentity();

    //    e->setRobustKernel(true);
    //e->setHuberWidth(0.01);

    optimizer.addEdge(e);
  }

  // set up SBA projections with some number of points

  true_points.clear();
  for (int i=0;i<num_points; ++i)
  {
    true_points.push_back(Vector3d((Sample::uniform()-0.5)*3,
                                   Sample::uniform()-0.5,
                                   Sample::uniform()+10));
  }


  // add point projections to this vertex
  for (size_t i=0; i<true_points.size(); ++i)
  {
    g2o::VertexSBAPointXYZ * v_p
        = new g2o::VertexSBAPointXYZ();


    v_p->setId(vertex_id++);
    v_p->setMarginalized(true);
    v_p->setEstimate(true_points.at(i)
        + Vector3d(Sample::gaussian(1),
                   Sample::gaussian(1),
                   Sample::gaussian(1)));

    optimizer.addVertex(v_p);

    for (size_t j=0; j<2; ++j)
      {
        Vector3d z;
        dynamic_cast<g2o::VertexSCam*>
          (optimizer.vertices().find(j)->second)
          ->mapPoint(z,true_points.at(i));

        if (z[0]>=0 && z[1]>=0 && z[0]<640 && z[1]<480)
        {
          z += Vector3d(Sample::gaussian(pix_noise),
                        Sample::gaussian(pix_noise),
                        Sample::gaussian(pix_noise/16.0));

          g2o::Edge_XYZ_VSC * e
              = new g2o::Edge_XYZ_VSC();

          e->vertices()[0]
              = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p);

          e->vertices()[1]
              = dynamic_cast<g2o::OptimizableGraph::Vertex*>
              (optimizer.vertices().find(j)->second);

          e->setMeasurement(z);
          //e->inverseMeasurement() = -z;
          e->information() = Matrix3d::Identity();

          //e->setRobustKernel(false);
          //e->setHuberWidth(1);

          optimizer.addEdge(e);
        }

      }
  } // done with adding projection points



  // move second cam off of its true position
  VertexSE3* vc = 
    dynamic_cast<VertexSE3*>(optimizer.vertices().find(1)->second);
  Eigen::Isometry3d cam = vc->estimate();
  cam.translation() = Vector3d(-0.1,0.1,0.2);
  vc->setEstimate(cam);
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  cout << "Initial chi2 = " << FIXED(optimizer.chi2()) << endl;

  optimizer.setVerbose(true);

  optimizer.optimize(20);

  cout << endl << "Second vertex should be near 0,0,1" << endl;
  cout <<  dynamic_cast<VertexSE3*>(optimizer.vertices().find(0)->second)
    ->estimate().translation().transpose() << endl;
  cout <<  dynamic_cast<VertexSE3*>(optimizer.vertices().find(1)->second)
    ->estimate().translation().transpose() << endl;
}
