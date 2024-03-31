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

#include <fstream>
#include <iostream>
#include <memory>
#include <optional>

#include "g2o/apps/g2o_simulator/sensor_odometry3d.h"
#include "g2o/apps/g2o_simulator/sensor_pointxyz.h"
#include "g2o/apps/g2o_simulator/sensor_pointxyz_depth.h"
#include "g2o/apps/g2o_simulator/sensor_pointxyz_disparity.h"
#include "g2o/apps/g2o_simulator/sensor_pose3d.h"
#include "g2o/apps/g2o_simulator/simulator.h"
#include "g2o/apps/g2o_simulator/simulator3d_base.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/slam3d/parameter_camera.h"

// #define _POSE_SENSOR_OFFSET
// #define _POSE_PRIOR_SENSOR

using std::cerr;

int main(int argc, char** argv) {
  g2o::CommandArgs arg;
  int nlandmarks;
  int simSteps;
  double worldSize;
  bool hasOdom;
  bool hasPoseSensor;
  bool hasPointSensor;
  bool hasPointDepthSensor;
  bool hasPointDisparitySensor;
  bool hasCompass;
  bool hasGPS;

  std::string outputFilename;
  arg.param("simSteps", simSteps, 100, "number of simulation steps");
  arg.param("nLandmarks", nlandmarks, 1000, "number of landmarks");
  arg.param("worldSize", worldSize, 25.0, "size of the world");
  arg.param("hasOdom", hasOdom, false, "the robot has an odometry");
  arg.param("hasPointSensor", hasPointSensor, false,
            "the robot has a point sensor");
  arg.param("hasPointDepthSensor", hasPointDepthSensor, false,
            "the robot has a point sensor");
  arg.param("hasPointDisparitySensor", hasPointDisparitySensor, false,
            "the robot has a point sensor");
  arg.param("hasPoseSensor", hasPoseSensor, false,
            "the robot has a pose sensor");
  arg.param("hasCompass", hasCompass, false, "the robot has a compass");
  arg.param("hasGPS", hasGPS, false, "the robot has a GPS");
  arg.paramLeftOver("graph-output", outputFilename, "simulator_out.g2o",
                    "graph file which will be written", true);

  arg.parseArgs(argc, argv);

  std::mt19937 generator;
  g2o::World world;
  for (int i = 0; i < nlandmarks; i++) {
    auto landmark = std::make_unique<g2o::WorldObjectTrackXYZ>();
    double x = g2o::sampleUniform(-.5, .5, &generator) * worldSize;
    double y = g2o::sampleUniform(-.5, .5, &generator) * worldSize;
    double z = g2o::sampleUniform(-.5, .5);
    landmark->vertex()->setEstimate(g2o::Vector3(x, y, z));
    world.addWorldObject(std::move(landmark));
  }

  {
    auto robot = std::make_unique<g2o::Robot3D>("myRobot");

    if (hasOdom) {
      auto odometrySensor = std::make_unique<g2o::SensorOdometry3D>("odometry");
      robot->addSensor(std::move(odometrySensor), world);
    }

    if (hasPointSensor) {
      auto pointSensor = std::make_unique<g2o::SensorPointXYZ>("pointSensor");
      pointSensor->setFov(M_PI / 4);
      Eigen::Isometry3d cameraPose;
      Eigen::Matrix3d R;
      R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
      pointSensor->setMaxRange(2.);
      cameraPose = R;
      cameraPose.translation() = g2o::Vector3(0., 0., 0.3);
      pointSensor->offsetParam()->setParam(cameraPose);
      robot->addSensor(std::move(pointSensor), world);
    }

    if (hasPointDisparitySensor) {
      auto disparitySensor =
          std::make_unique<g2o::SensorPointXYZDisparity>("disparitySensor");
      disparitySensor->setFov(M_PI / 4);
      disparitySensor->setMinRange(0.5);
      disparitySensor->setMaxRange(2.);
      g2o::CameraWithOffset cameraPose;
      cameraPose.offset().linear() << 0, 0, 1, -1, 0, 0, 0, -1, 0;
      cameraPose.offset().translation() = g2o::Vector3(0., 0., 0.3);
      disparitySensor->offsetParam()->setParam(cameraPose);
      robot->addSensor(std::move(disparitySensor), world);
    }

    if (hasPointDepthSensor) {
      auto depthSensor =
          std::make_unique<g2o::SensorPointXYZDepth>("depthSensor");
      depthSensor->setFov(M_PI / 4);
      depthSensor->setMinRange(0.5);
      depthSensor->setMaxRange(2.);
      g2o::CameraWithOffset cameraPose;
      cameraPose.offset().linear() << 0, 0, 1, -1, 0, 0, 0, -1, 0;
      cameraPose.offset().translation() = g2o::Vector3(0., 0., 0.3);
      depthSensor->offsetParam()->setParam(cameraPose);
      robot->addSensor(std::move(depthSensor), world);
    }

    if (hasPoseSensor) {
      auto poseSensor = std::make_unique<g2o::SensorPose3D>("poseSensor");
      poseSensor->setMaxRange(5);
      robot->addSensor(std::move(poseSensor), world);
    }

#ifdef _POSE_PRIOR_SENSOR
    SensorSE3Prior posePriorSensor("posePriorSensor");
    robot->addSensor(&posePriorSensor);
    {
      Eigen::Isometry3d cameraPose;
      Eigen::Matrix3d R;
      R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
      cameraPose = R;
      cameraPose.translation() = Vector3d(0., 0., 0.3);
      posePriorSensor.offsetParam()->setOffset(cameraPose);
    }
#endif

#ifdef _POSE_SENSOR_OFFSET
    SensorPose3DOffset poseSensor("poseSensor");
    poseSensor.setFov(M_PI / 4);
    poseSensor.setMinRange(0.5);
    poseSensor.setMaxRange(5);
    robot->addSensor(&poseSensor);
    if (0) {
      Eigen::Isometry3d cameraPose;
      Eigen::Matrix3d R;
      R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
      cameraPose = R;
      cameraPose.translation() = Vector3d(0., 0., 0.3);
      poseSensor.offsetParam1()->setOffset(cameraPose);
      poseSensor.offsetParam2()->setOffset(cameraPose);
    }
#endif
    world.addRobot(std::move(robot));
  }

  for (const auto& robot : world.robots()) {
    auto* rob2d = dynamic_cast<g2o::Robot3D*>(robot.get());
    if (!rob2d) continue;
    rob2d->move(world, Eigen::Isometry3d::Identity());
  }

  auto moveStraight = []() {
    Eigen::Isometry3d result;
    result = g2o::Translation3(1., 0., 0.);
    return result;
  };
  auto moveLeft = []() {
    Eigen::Isometry3d result;
    result = g2o::AngleAxis(M_PI / 2, g2o::Vector3::UnitZ());
    return result;
  };
  auto moveRight = []() {
    Eigen::Isometry3d result;
    result = g2o::AngleAxis(-M_PI / 2, g2o::Vector3::UnitZ());
    return result;
  };

  for (const auto& base_robot : world.robots()) {
    auto* robot = dynamic_cast<g2o::Robot3D*>(base_robot.get());
    if (!robot) continue;
    for (int i = 0; i < simSteps; i++) {
      cerr << "m";
      g2o::Vector3 dt;
      const Eigen::Isometry3d& pose = robot->pose();

      const std::optional<Eigen::Matrix3d> dtheta =
          [&]() -> std::optional<Eigen::Matrix3d> {
        if (pose.translation().x() < -.5 * worldSize) {
          return g2o::AngleAxis(0, g2o::Vector3::UnitZ()).toRotationMatrix();
        }
        if (pose.translation().x() > .5 * worldSize) {
          return g2o::AngleAxis(-M_PI, g2o::Vector3::UnitZ())
              .toRotationMatrix();
        }
        if (pose.translation().y() < -.5 * worldSize) {
          return g2o::AngleAxis(M_PI / 2, g2o::Vector3::UnitZ())
              .toRotationMatrix();
        }
        if (pose.translation().y() > .5 * worldSize) {
          return g2o::AngleAxis(-M_PI / 2, g2o::Vector3::UnitZ())
              .toRotationMatrix();
        }
        return std::nullopt;
      }();

      const Eigen::Isometry3d move = [&]() {
        if (dtheta.has_value()) {
          Eigen::Isometry3d result;
          result.linear() = pose.rotation().inverse() * dtheta.value();
          g2o::AngleAxis aa(result.linear());
          if (std::abs(aa.angle()) < std::numeric_limits<double>::epsilon()) {
            result.translation() = g2o::Vector3(1., 0., 0.);
          }
          return result;
        }
        constexpr double kPStraight = 0.7;
        constexpr double kPLeft = 0.15;
        const double sampled = g2o::sampleUniform();
        if (sampled < kPStraight) return moveStraight();
        if (sampled < kPStraight + kPLeft) return moveLeft();
        return moveRight();
      }();

      // select a random move of the robot
      robot->relativeMove(world, move);
      // do a sense
      cerr << "s";
      robot->sense(world);
    }
  }
  std::ofstream testStream(outputFilename.c_str());
  world.graph().save(testStream);
}
