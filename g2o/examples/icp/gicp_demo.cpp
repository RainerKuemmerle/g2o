// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <Eigen/StdVector>
#include <tr1/random>
#include <iostream>
#include <stdint.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/icp/types_icp.h"

using namespace Eigen;
using namespace std;
using namespace g2o;

// sampling distributions
  class Sample
  {

    static tr1::ranlux_base_01 gen_real;
    static tr1::mt19937 gen_int;
  public:
    static int uniform(int from, int to);

    static double uniform();

    static double gaussian(double sigma);
  };


 tr1::ranlux_base_01 Sample::gen_real;
  tr1::mt19937 Sample::gen_int;

  int Sample::uniform(int from, int to)
  {
    tr1::uniform_int<int> unif(from, to);
    int sam = unif(gen_int);
    return  sam;
  }

  double Sample::uniform()
  {
    std::tr1::uniform_real<double> unif(0.0, 1.0);
    double sam = unif(gen_real);
    return  sam;
  }

  double Sample::gaussian(double sigma)
  {
    std::tr1::normal_distribution<double> gauss(0.0, sigma);
    double sam = gauss(gen_real);
    return  sam;
  }


//
// set up simulated system with noise, optimize it
//

int main()
{
  double euc_noise = 0.01;       // noise in position, m
  //  double outlier_ratio = 0.1;


  SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  // variable-size block solver
  BlockSolverX::LinearSolverType * linearSolver
      = new LinearSolverCholmod<g2o
        ::BlockSolverX::PoseMatrixType>();


  BlockSolverX * solver_ptr
      = new BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  optimizer.setAlgorithm(solver);


  vector<Vector3d> true_points;
  for (size_t i=0;i<1000; ++i)
  {
    true_points.push_back(Vector3d((Sample::uniform()-0.5)*3,
                                   Sample::uniform()-0.5,
                                   Sample::uniform()+10));
  }


  // set up two poses
  int vertex_id = 0;
  for (size_t i=0; i<2; ++i)
  {
    // set up rotation and translation for this node
    Vector3d t(0,0,i);
    Quaterniond q;
    q.setIdentity();

    SE3Quat cam(q,t);           // camera pose

    // set up node
    VertexSE3 *vc = new VertexSE3();
    vc->setEstimate(cam);

    vc->setId(vertex_id);      // vertex id

    cerr << t.transpose() << " | " << q.coeffs().transpose() << endl;

    // set first cam pose fixed
    if (i==0)
      vc->setFixed(true);

    // add to optimizer
    optimizer.addVertex(vc);

    vertex_id++;                
  }

  // set up point matches
  for (size_t i=0; i<true_points.size(); ++i)
  {
    // get two poses
    VertexSE3* vp0 = 
      dynamic_cast<VertexSE3*>(optimizer.vertices().find(0)->second);
    VertexSE3* vp1 = 
      dynamic_cast<VertexSE3*>(optimizer.vertices().find(1)->second);

    // calculate the relative 3D position of the point
    Vector3d pt0,pt1;
    pt0 = vp0->estimate().inverse().map(true_points[i]);
    pt1 = vp1->estimate().inverse().map(true_points[i]);

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

    e->setVertex(0, vp0);      // first viewpoint

    e->setVertex(1, vp1);      // second viewpoint

    EdgeGICP meas;
    meas.pos0 = pt0;
    meas.pos1 = pt1;
    meas.normal0 = nm0;
    meas.normal1 = nm1;

    e->setMeasurement(meas);
    //        e->inverseMeasurement().pos() = -kp;
    
    meas = e->measurement();
    // use this for point-plane
    e->information() = meas.prec0(0.01);

    // use this for point-point 
    //    e->information().setIdentity();

    //    e->setRobustKernel(true);
    e->setHuberWidth(0.01);

    optimizer.addEdge(e);
  }

  // move second cam off of its true position
  VertexSE3* vc = 
    dynamic_cast<VertexSE3*>(optimizer.vertices().find(1)->second);
  SE3Quat cam = vc->estimate();
  cam.setTranslation(Vector3d(0,0,0.2));
  vc->setEstimate(cam);

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  cout << "Initial chi2 = " << FIXED(optimizer.chi2()) << endl;

  optimizer.setVerbose(true);

  optimizer.optimize(5);

  cout << endl << "Second vertex should be near 0,0,1" << endl;
  cout <<  dynamic_cast<VertexSE3*>(optimizer.vertices().find(0)->second)
    ->estimate().translation().transpose() << endl;
  cout <<  dynamic_cast<VertexSE3*>(optimizer.vertices().find(1)->second)
    ->estimate().translation().transpose() << endl;
}
