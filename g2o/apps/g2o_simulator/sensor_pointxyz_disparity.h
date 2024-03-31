// g2o - General Graph Optimization
// Copyright (C) 2011 G. Grisetti, R. Kuemmerle, W. Burgard
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

#ifndef G2O_SENSOR_POINTXYZ_DISPARITY_H_
#define G2O_SENSOR_POINTXYZ_DISPARITY_H_
#include "g2o/apps/g2o_simulator/simulator.h"
#include "g2o/types/slam3d/edge_se3_pointxyz_disparity.h"
#include "g2o/types/slam3d/parameter_camera.h"
#include "g2o_simulator_api.h"
#include "pointsensorparameters.h"
#include "simulator3d_base.h"

namespace g2o {

class G2O_SIMULATOR_API SensorPointXYZDisparity
    : public PointSensorParameters,
      public BinarySensor<Robot3D, EdgeSE3PointXYZDisparity,
                          WorldObjectTrackXYZ> {
 public:
  using RobotPoseType = PoseVertexType::EstimateType;
  explicit SensorPointXYZDisparity(const std::string& name);
  void sense(BaseRobot& robot, World& world) override;
  void addParameters(World& world) override;
  std::shared_ptr<ParameterCamera> offsetParam() { return offsetParam_; };
  void addNoise(EdgeType* e) override;

 protected:
  bool isVisible(WorldObjectType* to);
  RobotPoseType sensorPose_;
  std::shared_ptr<ParameterCamera> offsetParam_;
};

}  // namespace g2o

#endif
