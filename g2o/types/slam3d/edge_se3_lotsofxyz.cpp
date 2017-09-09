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

#include "edge_se3_lotsofxyz.h"

namespace g2o{

  EdgeSE3LotsOfXYZ::EdgeSE3LotsOfXYZ(){
    resize(0);
  }

  bool EdgeSE3LotsOfXYZ::setMeasurementFromState(){
    VertexSE3 * pose = static_cast<VertexSE3 *> (_vertices[0]);

    Eigen::Transform<double, 3, 1> poseinv = pose->estimate().inverse();

    for(unsigned int i=0; i<_observedPoints; i++){
      VertexPointXYZ * xyz = static_cast<VertexPointXYZ *> (_vertices[1+i]);
      //      const Vector3D &pt = xyz->estimate();
      Vector3D m = poseinv * xyz->estimate();

      unsigned int index = 3*i;
      _measurement[index] = m[0];
      _measurement[index+1] = m[1];
      _measurement[index+2] = m[2];
    }
    return true;
  }

  void EdgeSE3LotsOfXYZ::computeError(){
    VertexSE3 * pose = static_cast<VertexSE3 *> (_vertices[0]);

    for(unsigned int i=0; i<_observedPoints; i++){
      VertexPointXYZ * xyz = static_cast<VertexPointXYZ *> (_vertices[1+i]);
      Vector3D m = pose->estimate().inverse() * xyz->estimate();

      unsigned int index = 3*i;
      _error[index] = m[0] - _measurement[index];
      _error[index+1] = m[1] - _measurement[index+1];
      _error[index+2] = m[2] - _measurement[index+2];
    }
  }

  void EdgeSE3LotsOfXYZ::linearizeOplus(){
    g2o::VertexSE3 * pose = (g2o::VertexSE3 *) (_vertices[0]);

    // initialize Ji matrix
    MatrixXD Ji;
    unsigned int rows = 3*(_vertices.size()-1);
    Ji.resize(rows, 6);
    Ji.fill(0);

    Matrix3D poseRot = pose->estimate().inverse().rotation();

    for(unsigned int i=1; i<_vertices.size(); i++){
      g2o::VertexPointXYZ * point = (g2o::VertexPointXYZ *) (_vertices[i]);
      Vector3D Zcam = pose->estimate().inverse() * point->estimate();

      unsigned int index=3*(i-1);

      // Ji.block<3,3>(index,0) = -poseRot;
      Ji.block<3,3>(index,0) = -Matrix3D::Identity();

      Ji(index, 3) = -0.0;
      Ji(index, 4) = -2*Zcam(2);
      Ji(index, 5) = 2*Zcam(1);

      Ji(index+1, 3) = 2*Zcam(2);
      Ji(index+1, 4) = -0.0;
      Ji(index+1, 5) = -2*Zcam(0);

      Ji(index+2, 3) = -2*Zcam(1);
      Ji(index+2, 4) = 2*Zcam(0);
      Ji(index+2, 5) = -0.0;

      MatrixXD Jj;
      Jj.resize(rows, 3);
      Jj.fill(0);
      Jj.block<3,3>(index,0) = poseRot;

      _jacobianOplus[i] = Jj;
    }

    _jacobianOplus[0] = Ji;

  }

  bool EdgeSE3LotsOfXYZ::read(std::istream& is){
    is >> _observedPoints;

    setSize(_observedPoints + 1);

    // read the measurements
    for(unsigned int i=0; i<_observedPoints; i++){
      unsigned int index = 3*i;
      is >> _measurement[index] >> _measurement[index+1] >> _measurement[index+2];
    }

    // read the information matrix
    for(unsigned int i=0; i<_observedPoints*3; i++){
      // fill the "upper triangle" part of the matrix
      for(unsigned int j=i; j<_observedPoints*3; j++){
	is >> information()(i,j);
      }

      // fill the lower triangle part
      for(unsigned int j=0; j<i; j++){
	information()(i,j) = information()(j,i);
      }

    }
    return true;
  }



  bool EdgeSE3LotsOfXYZ::write(std::ostream& os) const{
    // write number of observed points
    os << "|| " << _observedPoints;

    // write measurements
    for(unsigned int i=0; i<_observedPoints; i++){
      unsigned int index = 3*i;
      os << " " << _measurement[index] << " " << _measurement[index+1] << " " << _measurement[index+2];
    }

    // write information matrix
    for(unsigned int i=0; i<_observedPoints*3; i++){
      for(unsigned int j=i; j<_observedPoints*3; j++){
	os << " " << information()(i,j);
      }
    }
    return os.good();
  }



  void EdgeSE3LotsOfXYZ::initialEstimate(const OptimizableGraph::VertexSet& fixed, OptimizableGraph::Vertex* toEstimate){
    (void) toEstimate;

    assert(initialEstimatePossible(fixed, toEstimate) && "Bad vertices specified");

    VertexSE3 * pose = static_cast<VertexSE3 *>(_vertices[0]);

#ifdef _MSC_VER
	std::vector<bool> estimate_this(_observedPoints, true);
#else
    bool estimate_this[_observedPoints];
    for(unsigned int i=0; i<_observedPoints; i++){
      estimate_this[i] = true;
    }
#endif

    for(std::set<HyperGraph::Vertex*>::iterator it=fixed.begin(); it!=fixed.end(); it++){
      for(unsigned int i=1; i<_vertices.size(); i++){
	VertexPointXYZ * vert = static_cast<VertexPointXYZ *>(_vertices[i]);
	if(vert->id() == (*it)->id()) estimate_this[i-1] = false;
      }
    }

    for(unsigned int i=1; i<_vertices.size(); i++){
      if(estimate_this[i-1]){
	unsigned int index = 3*(i-1);
	Vector3D submeas(_measurement[index], _measurement[index+1], _measurement[index+2]);
	VertexPointXYZ * vert = static_cast<VertexPointXYZ *>(_vertices[i]);
	vert->setEstimate(pose->estimate() * submeas);
      }
    }
  }



  double EdgeSE3LotsOfXYZ::initialEstimatePossible(const OptimizableGraph::VertexSet& fixed, OptimizableGraph::Vertex* toEstimate){
    (void) toEstimate;

    for(std::set<HyperGraph::Vertex *>::iterator it=fixed.begin(); it!=fixed.end(); it++){
      if(_vertices[0]->id() == (*it)->id()){
	return 1.0;
      }
    }

    return -1.0;
  }
}
