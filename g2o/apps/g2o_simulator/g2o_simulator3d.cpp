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

#include "g2o/core/optimizable_graph.h"
#include "g2o/simulator/simulator3d_base.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/logger.h"

int main(int argc, char** argv) {
  g2o::CommandArgs arg;
  g2o::Simulator3D::Config simulator_config;

  std::string outputFilename;
  arg.param("simSteps", simulator_config.simSteps, 100,
            "number of simulation steps");
  arg.param("nLandmarks", simulator_config.nlandmarks, 1000,
            "number of landmarks");
  arg.param("worldSize", simulator_config.worldSize, 25.0, "size of the world");
  arg.param("hasOdom", simulator_config.hasOdom, false,
            "the robot has an odometry");
  arg.param("hasPointSensor", simulator_config.hasPointSensor, false,
            "the robot has a point sensor");
  arg.param("hasPointDepthSensor", simulator_config.hasPointDepthSensor, false,
            "the robot has a point sensor");
  arg.param("hasPointDisparitySensor", simulator_config.hasPointDisparitySensor,
            false, "the robot has a point sensor");
  arg.param("hasPoseSensor", simulator_config.hasPoseSensor, false,
            "the robot has a pose sensor");
  arg.param("hasCompass", simulator_config.hasCompass, false,
            "the robot has a compass");
  arg.param("hasGPS", simulator_config.hasGPS, false, "the robot has a GPS");
  arg.paramLeftOver("graph-output", outputFilename, "",
                    "graph file which will be written", true);

  arg.parseArgs(argc, argv);

  g2o::Simulator3D simulator(std::move(simulator_config));
  G2O_INFO("Setting up");
  simulator.setup();
  G2O_INFO("Simulate");
  simulator.simulate();

  if (outputFilename.empty()) {
    G2O_INFO("Saving to stdout");
    simulator.world().graph().save(std::cout);
  } else {
    G2O_INFO("Saving to {}", outputFilename);
    std::ofstream testStream(outputFilename);
    simulator.world().graph().save(testStream);
  }

  return EXIT_SUCCESS;
}
