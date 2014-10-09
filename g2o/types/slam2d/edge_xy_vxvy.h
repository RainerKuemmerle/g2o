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

#ifndef G2O_EDGE_XY_VXVY_H
#define G2O_EDGE_XY_VXVY_H

#include "g2o/config.h"
#include "vertex_xy_vxvy.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o_types_slam2d_api.h"

using namespace Eigen;

namespace g2o {

  class G2O_TYPES_SLAM2D_API EdgeXY_VXVY : public BaseBinaryEdge<4, Eigen::Vector4d, VertexXY_VXVY, VertexXY_VXVY>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeXY_VXVY();

      void computeError()
      {
        const VertexXY_VXVY* m1 = static_cast<const VertexXY_VXVY*>(_vertices[0]); // moving landmark position 1
        const VertexXY_VXVY* m2 = static_cast<const VertexXY_VXVY*>(_vertices[1]); // moving landmark position 2
        //_error = (v1->estimate().inverse() * l2->estimate()) - _measurement;
	
	unsigned long long timestamp2, timestamp1;
	timestamp1 = m1->timestamp;
	timestamp2 = m2->timestamp;
	
	double deltaT = ((double)abs(timestamp2 - timestamp1))/1000000;
	//std::cout<<deltaT<<std::endl;
	
	Matrix4d F;
	F(0, 0) =  1;  F(0, 1) = 0;   F(0, 2) = deltaT; F(0, 3) = 0;
	F(1, 0) =  0;  F(1, 1) = 1;   F(1, 2) = 0; F(1, 3) = deltaT;
	F(2, 0) =  0;  F(2, 1) = 0;   F(2, 2) = 1; F(2, 3) = 0;
	F(3, 0) =  0;  F(3, 1) = 0;   F(3, 2) = 0; F(3, 3) = 1;  	
	_error = (m2->estimate() - F*m1->estimate())- _measurement;

      }

      virtual bool setMeasurementData(const double* d){
  _measurement[0]=d[0];
  _measurement[1]=d[1];
  _measurement[2]=d[2];
  _measurement[3]=d[3];
  return true;
      }

      virtual bool getMeasurementData(double* d) const{
  d[0] = _measurement[0];
  d[1] = _measurement[1];
  d[2] = _measurement[2];
  d[3] = _measurement[3];  
  return true;
      }
      
      virtual int measurementDimension() const {return 4;}

      virtual bool setMeasurementFromState(){
        const VertexXY_VXVY* m1 = static_cast<const VertexXY_VXVY*>(_vertices[0]);
        const VertexXY_VXVY* m2 = static_cast<const VertexXY_VXVY*>(_vertices[1]);
	
	unsigned long long timestamp2, timestamp1;
	timestamp1 = m1->timestamp;
	timestamp2 = m2->timestamp;
	
	double deltaT = ((double)abs(timestamp2 - timestamp1))/1000000;
	//std::cout<<deltaT<<std::endl;
	
	Matrix4d F;
	F(0, 0) =  1;  F(0, 1) = 0;   F(0, 2) = deltaT; F(0, 3) = 0;
	F(1, 0) =  0;  F(1, 1) = 1;   F(1, 2) = 0; F(1, 3) = deltaT;
	F(2, 0) =  0;  F(2, 1) = 0;   F(2, 2) = 1; F(2, 3) = 0;
	F(3, 0) =  0;  F(3, 1) = 0;   F(3, 2) = 0; F(3, 3) = 1;  	
  _measurement = (m2->estimate() - F*m1->estimate());
  return true;
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) { (void) to; return (from.count(_vertices[0]) == 1 ? 1.0 : -1.0);}
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
      virtual void linearizeOplus();
#endif
  };

  class G2O_TYPES_SLAM2D_API EdgeXY_VXVYWriteGnuplotAction: public WriteGnuplotAction {
  public:
    EdgeXY_VXVYWriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };

#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM2D_API EdgeXY_VXVYDrawAction: public DrawAction{
  public:
    EdgeXY_VXVYDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };
#endif

} // end namespace

#endif
