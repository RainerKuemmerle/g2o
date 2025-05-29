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

#include <cstdlib>
#include <fstream>
#include <iostream>

#include "CLI/CLI.hpp"
#include "g2o/core/optimizable_graph.h"
#include "g2o/simulator/simulator3d_base.h"
#include "g2o/stuff/logger.h"

int main(int argc, char** argv) {
  g2o::Simulator3D::Config simulator_config;
  simulator_config.simSteps = 100;
  simulator_config.nlandmarks = 1000;
  simulator_config.worldSize = 25.0;

  CLI::App app{"g2o Simulator 3D"};
  argv = app.ensure_utf8(argv);

  std::string outputFilename;
  app.add_flag("--sim_steps", simulator_config.simSteps,
               "number of simulation steps")
      ->check(CLI::PositiveNumber);
  app.add_flag("--num_landmarks", simulator_config.nlandmarks,
               "number of landmarks")
      ->check(CLI::NonNegativeNumber);
  app.add_flag("--world_size", simulator_config.worldSize, "size of the world")
      ->check(CLI::NonNegativeNumber);
  app.add_flag("--has_odom", simulator_config.hasOdom,
               "the robot has an odometry");
  app.add_flag("--has_point_sensor", simulator_config.hasPointSensor,
               "the robot has a point sensor");
  app.add_flag("--has_point_depth_sensor", simulator_config.hasPointDepthSensor,
               "the robot has a point sensor");
  app.add_flag("--has_point_disparity_sensor",
               simulator_config.hasPointDisparitySensor,
               "the robot has a point sensor");
  app.add_flag("has_pose_sensor", simulator_config.hasPoseSensor,
               "the robot has a pose sensor");
  app.add_flag("--has_compass", simulator_config.hasCompass,
               "the robot has a compass");
  app.add_flag("--has_gps", simulator_config.hasGPS, "the robot has a GPS");
  app.add_option("graph-output", outputFilename,
                 "graph file which will be written ('-' for stdout)")
      ->required();

  CLI11_PARSE(app, argc, argv);

  G2O_INFO("Running 3D simulator");
  g2o::Simulator3D simulator(std::move(simulator_config));
  G2O_INFO("Setting up");
  simulator.setup();
  G2O_INFO("Simulate");
  simulator.simulate();

  if (outputFilename == "-") {
    G2O_INFO("Saving to stdout");
    simulator.world().graph().save(std::cout);
  } else {
    G2O_INFO("Saving to {}", outputFilename);
    std::ofstream testStream(outputFilename);
    simulator.world().graph().save(testStream);
  }

  return EXIT_SUCCESS;
}
