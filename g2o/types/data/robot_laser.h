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

#ifndef G2O_ROBOT_LASER_H
#define G2O_ROBOT_LASER_H

#include "raw_laser.h"
#include "g2o_types_data_api.h"
#include "g2o/core/hyper_graph_action.h"

namespace g2o {

  /**
   * \brief laser measurement obtained by a robot
   *
   * A laser measurement obtained by a robot. The measurement is equipped with a pose of the robot at which
   * the measurement was taken. The read/write function correspond to the CARMEN logfile format.
   */
  class G2O_TYPES_DATA_API RobotLaser : public RawLaser
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      RobotLaser();
      ~RobotLaser();

      virtual bool write(std::ostream& os) const;
      virtual bool read(std::istream& is);

      SE2 laserPose() const { return _odomPose * _laserParams.laserPose;} 
      const SE2& odomPose() const { return _odomPose;}
      void setOdomPose(const SE2& odomPose);

    protected:
      SE2 _odomPose;
      //! velocities and safety distances of the robot.
      double _laserTv, _laserRv, _forwardSafetyDist, _sideSaftyDist, _turnAxis;
  };

#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_DATA_API RobotLaserDrawAction: public DrawAction{
  public:
    RobotLaserDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_ );
  protected:
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
    IntProperty* _beamsDownsampling;
    FloatProperty* _pointSize;
    FloatProperty* _maxRange;
  };
#endif

}

#endif
