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
#include "g2o/simulator/simulator.h"
#include "g2o/simulator/simulator2d_base.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/logger.h"

int main(int argc, char** argv) {
  g2o::CommandArgs arg;
  g2o::Simulator2D::Config simulator_config;

  G2O_INFO("Running 2D simulator");

  std::string outputFilename;
  arg.param("nlandmarks", simulator_config.nlandmarks, 100,
            "number of landmarks in the map");
  arg.param("nSegments", simulator_config.nSegments, 1000,
            "number of segments");
  arg.param("segmentGridSize", simulator_config.segmentGridSize, 50,
            "number of cells of the grid where to align the segments");
  arg.param("minSegmentLength", simulator_config.minSegmentLength, 0.5,
            "minimal Length of a segment in the world");
  arg.param("maxSegmentLength", simulator_config.maxSegmentLength, 3,
            "maximal Length of a segment in the world");

  arg.param("simSteps", simulator_config.simSteps, 100,
            "number of simulation steps");
  arg.param("worldSize", simulator_config.worldSize, 25.0, "size of the world");
  arg.param("hasOdom", simulator_config.hasOdom, false,
            "the robot has an odometry");
  arg.param("hasPointSensor", simulator_config.hasPointSensor, false,
            "the robot has a point sensor");
  arg.param("hasPointBearingSensor", simulator_config.hasPointBearingSensor,
            false, "the robot has a point bearing sensor");
  arg.param("hasPoseSensor", simulator_config.hasPoseSensor, false,
            "the robot has a pose sensor");
  arg.param("hasCompass", simulator_config.hasCompass, false,
            "the robot has a compass");
  arg.param("hasGPS", simulator_config.hasGPS, false, "the robot has a GPS");
  arg.param("hasSegmentSensor", simulator_config.hasSegmentSensor, false,
            "the robot has a segment sensor");
  arg.paramLeftOver("graph-output", outputFilename, "",
                    "graph file which will be written", true);

  arg.parseArgs(argc, argv);

  g2o::Simulator2D simulator(std::move(simulator_config));
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
