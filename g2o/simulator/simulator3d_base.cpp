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

#include "simulator3d_base.h"

#include <memory>
#include <optional>

#include "g2o/core/eigen_types.h"
#include "g2o/simulator/sensor_odometry3d.h"
#include "g2o/simulator/sensor_pointxyz.h"
#include "g2o/simulator/sensor_pointxyz_depth.h"
#include "g2o/simulator/sensor_pointxyz_disparity.h"
#include "g2o/simulator/sensor_pose3d.h"
#include "g2o/simulator/sensor_se3_prior.h"
#include "g2o/stuff/logger.h"

// TODO(Rainer): Figure out if this makes sense
// #define POSE_SENSOR_OFFSET

namespace g2o {

Simulator3D::Simulator3D(Simulator3D::Config&& config) : config(config) {}

void Simulator3D::setup() {
  for (int i = 0; i < config.nlandmarks; i++) {
    auto landmark = std::make_unique<WorldObjectTrackXYZ>();
    double x = sampleUniform(-.5, .5, &generator_) * config.worldSize;
    double y = sampleUniform(-.5, .5, &generator_) * config.worldSize;
    double z = sampleUniform(-.5, .5);
    landmark->vertex()->setEstimate(Vector3(x, y, z));
    world_.addWorldObject(std::move(landmark));
  }

  auto robot = std::make_unique<Robot3D>("myRobot");

  if (config.hasOdom) {
    auto odometrySensor = std::make_unique<SensorOdometry3D>("odometry");
    robot->addSensor(std::move(odometrySensor), world_);
  }

  if (config.hasPointSensor) {
    auto pointSensor = std::make_unique<SensorPointXYZ>("pointSensor");
    pointSensor->setFov(M_PI / 4);
    Isometry3 cameraPose;
    Matrix3 R;
    R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    pointSensor->setMaxRange(2.);
    cameraPose = R;
    cameraPose.translation() = Vector3(0., 0., 0.3);
    pointSensor->addParameters(world_);
    pointSensor->offsetParam()->setParam(cameraPose);
    robot->addSensor(std::move(pointSensor), world_);
  }

  if (config.hasPointDisparitySensor) {
    auto disparitySensor =
        std::make_unique<SensorPointXYZDisparity>("disparitySensor");
    disparitySensor->setFov(M_PI / 4);
    disparitySensor->setMinRange(0.5);
    disparitySensor->setMaxRange(2.);
    CameraWithOffset cameraPose;
    cameraPose.offset().linear() << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    cameraPose.offset().translation() = Vector3(0., 0., 0.3);
    disparitySensor->addParameters(world_);
    disparitySensor->offsetParam()->setParam(cameraPose);
    robot->addSensor(std::move(disparitySensor), world_);
  }

  if (config.hasPointDepthSensor) {
    auto depthSensor = std::make_unique<SensorPointXYZDepth>("depthSensor");
    depthSensor->setFov(M_PI / 4);
    depthSensor->setMinRange(0.5);
    depthSensor->setMaxRange(2.);
    CameraWithOffset cameraPose;
    cameraPose.offset().linear() << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    cameraPose.offset().translation() = Vector3(0., 0., 0.3);
    depthSensor->addParameters(world_);
    depthSensor->offsetParam()->setParam(cameraPose);
    robot->addSensor(std::move(depthSensor), world_);
  }

  if (config.hasPoseSensor) {
    auto poseSensor = std::make_unique<SensorPose3D>("poseSensor");
    poseSensor->setMaxRange(5);
    robot->addSensor(std::move(poseSensor), world_);
  }

  if (config.hasGPS) {
    auto gpsSensor = std::make_unique<SensorSE3Prior>("posePriorSensor");
    Isometry3 cameraPose = Isometry3::Identity();
    cameraPose.linear() << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    cameraPose.translation() = Vector3(0., 0., 0.3);
    gpsSensor->addParameters(world_);
    gpsSensor->offsetParam()->setParam(cameraPose);
    robot->addSensor(std::move(gpsSensor), world_);
  }

#ifdef POSE_SENSOR_OFFSET
  SensorPose3DOffset poseSensor("poseSensor");
  poseSensor.setFov(M_PI / 4);
  poseSensor.setMinRange(0.5);
  poseSensor.setMaxRange(5);
  robot->addSensor(&poseSensor);
  if (0) {
    Isometry3 cameraPose;
    Matrix3 R;
    R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    cameraPose = R;
    cameraPose.translation() = Vector3d(0., 0., 0.3);
    poseSensor.offsetParam1()->setOffset(cameraPose);
    poseSensor.offsetParam2()->setOffset(cameraPose);
  }
#endif
  world_.addRobot(std::move(robot));
}
void Simulator3D::simulate() {
  const Config& sim_conf = config;

  for (const auto& robot : world_.robots()) {
    auto* rob2d = dynamic_cast<Robot3D*>(robot.get());
    if (!rob2d) continue;
    rob2d->move(world_, Isometry3::Identity());
  }

  const Isometry3 moveStraight = []() {
    Isometry3 result;
    result = Translation3(1., 0., 0.);
    return result;
  }();
  const Isometry3 moveLeft = []() {
    Isometry3 result;
    result = AngleAxis(M_PI / 2, Vector3::UnitZ());
    return result;
  }();
  const Isometry3 moveRight = []() {
    Isometry3 result;
    result = AngleAxis(-M_PI / 2, Vector3::UnitZ());
    return result;
  }();

  for (const auto& base_robot : world_.robots()) {
    auto* robot = dynamic_cast<Robot3D*>(base_robot.get());
    if (!robot) continue;
    for (int i = 0; i < config.simSteps; i++) {
      G2O_TRACE("move");
      Vector3 dt;
      const Isometry3& pose = robot->pose();

      const std::optional<Matrix3> dtheta =
          [&pose, &sim_conf]() -> std::optional<Matrix3> {
        if (pose.translation().x() < -.5 * sim_conf.worldSize) {
          return AngleAxis(0, Vector3::UnitZ()).toRotationMatrix();
        }
        if (pose.translation().x() > .5 * sim_conf.worldSize) {
          return AngleAxis(-M_PI, Vector3::UnitZ()).toRotationMatrix();
        }
        if (pose.translation().y() < -.5 * sim_conf.worldSize) {
          return AngleAxis(M_PI / 2, Vector3::UnitZ()).toRotationMatrix();
        }
        if (pose.translation().y() > .5 * sim_conf.worldSize) {
          return AngleAxis(-M_PI / 2, Vector3::UnitZ()).toRotationMatrix();
        }
        return std::nullopt;
      }();

      const Isometry3 move = [&pose, &dtheta, &moveStraight, &moveLeft,
                              &moveRight]() {
        if (dtheta.has_value()) {
          Isometry3 result;
          result.linear() = pose.rotation().inverse() * dtheta.value();
          AngleAxis aa(result.linear());
          if (std::abs(aa.angle()) < std::numeric_limits<double>::epsilon()) {
            result.translation() = Vector3(1., 0., 0.);
          }
          return result;
        }
        constexpr double kPStraight = 0.7;
        constexpr double kPLeft = 0.15;
        const double sampled = sampleUniform();
        if (sampled < kPStraight) return moveStraight;       // NOLINT
        if (sampled < kPStraight + kPLeft) return moveLeft;  // NOLINT
        return moveRight;                                    // NOLINT
      }();

      // select a random move of the robot
      robot->relativeMove(world_, move);
      // do a sense
      G2O_TRACE("sense");
      robot->sense(world_);
    }
  }
  finalize();
}

}  // namespace g2o
