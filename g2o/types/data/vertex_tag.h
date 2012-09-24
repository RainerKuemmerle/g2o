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

#ifndef G2O_VERTEX_TAG_H
#define G2O_VERTEX_TAG_H

#include "robot_data.h"
#include "g2o_types_data_api.h"
#include "g2o/core/hyper_graph_action.h"

namespace g2o {

  /**
   * \brief string tag to be attached to a vertex
   *
   * A laser measurement obtained by a robot. The measurement is equipped with a pose of the robot at which
   * the measurement was taken. The read/write function correspond to the CARMEN logfile format.
   */
  class G2O_TYPES_DATA_API VertexTag : public RobotData
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VertexTag();
      ~VertexTag();

      virtual bool write(std::ostream& os) const;
      virtual bool read(std::istream& is);
      
      const std::string name() const { return _name;} 
      void setName(const std::string& name_) {_name=name_;}
      const Eigen::Vector3f& position() const {return _position;}
      void setPosition( const Eigen::Vector3f& p) {_position = p;}
    protected:
      std::string _name;
      Eigen::Vector3f _position;
      Eigen::Vector3f _odom2d;
  };

 #ifdef G2O_HAVE_OPENGL 
  class G2O_TYPES_DATA_API VertexTagDrawAction: public DrawAction{
  public:
    VertexTagDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
            HyperGraphElementAction::Parameters* params_ );
  protected:
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
    DoubleProperty* _textSize;
  };
#endif

}

#endif
