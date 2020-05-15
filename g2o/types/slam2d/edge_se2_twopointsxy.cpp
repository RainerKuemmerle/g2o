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

#include "edge_se2_twopointsxy.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif


namespace g2o{

  EdgeSE2TwoPointsXY::EdgeSE2TwoPointsXY() : BaseMultiEdge<4,Vector4>(){
    resize(3);
  }

  void EdgeSE2TwoPointsXY::computeError(){
    VertexSE2 * pose = static_cast<VertexSE2 *> (_vertices[0]);
    VertexPointXY * xy1 = static_cast<VertexPointXY *> (_vertices[1]);
    VertexPointXY * xy2 = static_cast<VertexPointXY *> (_vertices[2]);


    Vector2 m1 = pose->estimate().inverse() * xy1->estimate();
    Vector2 m2 = pose->estimate().inverse() * xy2->estimate();

    _error[0] = m1[0] - _measurement[0];
    _error[1] = m1[1] - _measurement[1];
    _error[2] = m2[0] - _measurement[2];
    _error[3] = m2[1] - _measurement[3];
  }

  bool EdgeSE2TwoPointsXY::read(std::istream& is){
    is >> _measurement[0] >> _measurement[1] >> _measurement[2] >> _measurement[3];
    is >> information()(0,0) >> information()(0,1) >> information()(0,2) >> information()(0,3) >> information()(1,1) >> information()(1,2) >> information()(1,3) >> information()(2,2) >> information()(2,3) >> information()(3,3);
    information()(1,0) = information()(0,1);
    information()(2,0) = information()(0,2);
    information()(2,1) = information()(1,2);
    information()(3,0) = information()(0,3);
    information()(3,1) = information()(1,3);
    information()(3,2) = information()(2,3);
    return true;
  }

  bool EdgeSE2TwoPointsXY::write(std::ostream& os) const{
    os << measurement()[0] << " " << measurement()[1] << " " << measurement()[2] << " "  << measurement()[3] << " ";
    os << information()(0,0) << " " << information()(0,1) << " " << information()(0,2) << " " << information()(0,3) << " " << information()(1,1) << " " << information()(1,2) << " " << information()(1,3) << " " << information()(2,2) << " " << information()(2,3) << " " << information()(3,3);
    return os.good();
  }


  void EdgeSE2TwoPointsXY::initialEstimate(const OptimizableGraph::VertexSet& fixed, OptimizableGraph::Vertex* toEstimate){
    (void) toEstimate;

    assert(initialEstimatePossible(fixed, toEstimate) && "Bad vertices specified");

    VertexSE2 * pose = static_cast<VertexSE2 *>(_vertices[0]);
    VertexPointXY * v1 = static_cast<VertexPointXY *>(_vertices[1]);
    VertexPointXY * v2 = static_cast<VertexPointXY *>(_vertices[2]);

    bool estimatev1 = true;
    bool estimatev2 = true;

    for(std::set<HyperGraph::Vertex*>::iterator it=fixed.begin(); it!=fixed.end(); ++it){
      if(v1->id() == (*it)->id()){
        estimatev1 = false;
      }
      else if(v2->id() == (*it)->id()){
        estimatev2 = false;
      }
    }

    if(estimatev1){
      Vector2 submeas(_measurement[0], _measurement[1]);
      v1->setEstimate(pose->estimate() * submeas);
    }

    if(estimatev2){
      Vector2 submeas(_measurement[2], _measurement[3]);
      v2->setEstimate(pose->estimate() * submeas);
    }
  }


  number_t EdgeSE2TwoPointsXY::initialEstimatePossible(const OptimizableGraph::VertexSet& fixed, OptimizableGraph::Vertex* toEstimate){
    (void) toEstimate;

    for(std::set<HyperGraph::Vertex *>::iterator it=fixed.begin(); it!=fixed.end(); ++it){
      if(_vertices[0]->id() == (*it)->id()){
        return 1.0;
      }
    }
    return -1.0;
  }


  bool EdgeSE2TwoPointsXY::setMeasurementFromState(){
    VertexSE2 * pose = static_cast<VertexSE2 *> (_vertices[0]);
    VertexPointXY * xy1 = static_cast<VertexPointXY *> (_vertices[1]);
    VertexPointXY * xy2 = static_cast<VertexPointXY *> (_vertices[2]);


    Vector2 m1 = pose->estimate().inverse() * xy1->estimate();
    Vector2 m2 = pose->estimate().inverse() * xy2->estimate();

    _measurement[0] = m1[0];
    _measurement[1] = m1[1];
    _measurement[2] = m2[0];
    _measurement[3] = m2[1];
    return true;
  }

} // end namespace g2o
