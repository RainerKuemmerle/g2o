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

#ifndef G2O_VERTEX_XY_VXVY_H
#define G2O_VERTEX_XY_VXVY_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "se2.h"
#include "g2o_types_slam2d_api.h"

using namespace Eigen;

namespace g2o 
{

  /**
   * \brief moving spherical landmark (no orientation) position (2D) + velocity(2D) hence a 4D Vertex, (x,y,Vx,Vy)
   */
  class G2O_TYPES_SLAM2D_API VertexXY_VXVY : public BaseVertex<4, Eigen::Vector4d>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      VertexXY_VXVY();

      virtual void setToOriginImpl() 
      {
        _estimate.setZero();
      }

      virtual bool setEstimateDataImpl(const double* est)
      {
	_estimate[0] = est[0];
	_estimate[1] = est[1];
	_estimate[2] = est[2];
	_estimate[3] = est[3];
	return true;
      }

      virtual bool getEstimateData(double* est) const
      {
	est[0] = _estimate[0];
	est[1] = _estimate[1];
	est[2] = _estimate[2];
	est[3] = _estimate[3];
	return true;
      }
	  
      virtual int estimateDimension() const 
      { 
	return 4;
      }

      virtual bool setMinimalEstimateDataImpl(const double* est)
      {
	return setEstimateData(est);
      }

      virtual bool getMinimalEstimateData(double* est) const
      {
	return getEstimateData(est);
      }
	  
      virtual int minimalEstimateDimension() const 
      { 
	return 4;
      }

      virtual void oplusImpl(const double* update)
      {
// 	_estimate[0] += (update[0]*0.033 + update[2]*0.033*0.033);
// 	_estimate[1] += (update[1]*0.033 + update[3]*0.033*0.033);
	///@TODO This has to be checked for correctness
	//std::cout<<update[0]<<" "<<update[1]<<" "<<update[2]<<" "<<update[3]<<std::endl;
// 	_estimate[0] += (update[0] + 0.5*update[2]*0.033);
// 	_estimate[1] += (update[1] + 0.5*update[3]*0.033);	
// 	_estimate[2] += (update[0]/0.033 + update[2]);
// 	_estimate[3] += (update[1]/0.033 + update[3]);
// 	
	_estimate[0] += (update[0]);
	_estimate[1] += (update[1]);	
	_estimate[2] += ((update[2])==(update[2]) ? (update[2]) : 0);
	_estimate[3] += ((update[3])==(update[3]) ? (update[3]) : 0);
	
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;
    
      unsigned long long timestamp;
      double ballZCoordinate;

  };

  class G2O_TYPES_SLAM2D_API VertexXY_VXVYWriteGnuplotAction: public WriteGnuplotAction 
  {
    public:
	VertexXY_VXVYWriteGnuplotAction();
	virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_);
  };

#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM2D_API VertexXY_VXVYDrawAction: public DrawAction
  {
      public:
	VertexXY_VXVYDrawAction();
	virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_);
      protected:
	FloatProperty *_pointSize;
	virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
  };
#endif

}

#endif
