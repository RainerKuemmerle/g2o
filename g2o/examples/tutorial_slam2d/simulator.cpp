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

#include "simulator.h"

#include <cmath>
#include <iostream>
#include <map>

#include "g2o/stuff/sampler.h"

using std::cerr;
using std::endl;

namespace g2o {
namespace tutorial {

#ifdef _MSC_VER
inline double round(double number) {
  return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
}
#endif

using LandmarkGrid = std::map<int, std::map<int, Simulator::LandmarkPtrVector>>;

Simulator::Simulator() {
  const time_t seed = time(nullptr);
  Sampler::seedRand(static_cast<unsigned int>(seed));
}

void Simulator::simulate(int numPoses, const SE2& sensorOffset) {
  // simulate a robot observing landmarks while travelling on a grid
  const int steps = 5;
  const double stepLen = 1.0;
  const int boundArea = 50;

  const double maxSensorRangeLandmarks = 2.5 * stepLen;

  const int landMarksPerSquareMeter = 1;
  const double observationProb = 0.8;

  const int landmarksRange = 2;

  const Eigen::Vector2d transNoise(0.05, 0.01);
  const double rotNoise = DEG2RAD(2.);
  const Eigen::Vector2d landmarkNoise(0.05, 0.05);

  const Eigen::Vector2d bound(boundArea, boundArea);

  Eigen::VectorXd probLimits;
  probLimits.resize(kMoNumElems);
  for (int i = 0; i < probLimits.size(); ++i)
    probLimits[i] = (i + 1) / static_cast<double>(kMoNumElems);

  const Eigen::Matrix3d covariance =
      Eigen::Vector3d(transNoise[0] * transNoise[0],
                      transNoise[1] * transNoise[1], rotNoise * rotNoise)
          .asDiagonal();
  const Eigen::Matrix3d information = covariance.inverse();

  const SE2 maxStepTransf(stepLen * steps, 0, 0);
  Simulator::PosesVector& poses = poses_;
  poses.clear();
  LandmarkVector& landmarks = landmarks_;
  landmarks.clear();
  Simulator::GridPose firstPose;
  firstPose.id = 0;
  firstPose.truePose = SE2(0, 0, 0);
  firstPose.simulatorPose = SE2(0, 0, 0);
  poses.push_back(firstPose);
  cerr << "Simulator: sampling nodes ...";
  while (static_cast<int>(poses.size()) < numPoses) {
    // add straight motions
    for (int i = 1; i < steps && static_cast<int>(poses.size()) < numPoses;
         ++i) {
      const Simulator::GridPose nextGridPose = generateNewPose(
          poses.back(), SE2(stepLen, 0, 0), transNoise, rotNoise);
      poses.push_back(nextGridPose);
    }
    if (static_cast<int>(poses.size()) == numPoses) break;

    // sample a new motion direction
    const double sampleMove = Sampler::uniformRand(0., 1.);
    int motionDirection = 0;
    while (probLimits[motionDirection] < sampleMove &&
           motionDirection + 1 < kMoNumElems) {
      motionDirection++;
    }

    SE2 nextMotionStep = getMotion(motionDirection, stepLen);
    Simulator::GridPose nextGridPose =
        generateNewPose(poses.back(), nextMotionStep, transNoise, rotNoise);

    // check whether we will walk outside the boundaries in the next iteration
    SE2 nextStepFinalPose = nextGridPose.truePose * maxStepTransf;
    if (fabs(nextStepFinalPose.translation().x()) >= bound[0] ||
        fabs(nextStepFinalPose.translation().y()) >= bound[1]) {
      // cerr << "b";
      // will be outside boundaries using this
      for (int i = 0; i < kMoNumElems; ++i) {
        nextMotionStep = getMotion(i, stepLen);
        nextGridPose =
            generateNewPose(poses.back(), nextMotionStep, transNoise, rotNoise);
        nextStepFinalPose = nextGridPose.truePose * maxStepTransf;
        if (fabs(nextStepFinalPose.translation().x()) < bound[0] &&
            fabs(nextStepFinalPose.translation().y()) < bound[1])
          break;
      }
    }

    poses.push_back(nextGridPose);
  }
  cerr << "done." << endl;

  // creating landmarks along the trajectory
  cerr << "Simulator: Creating landmarks ... ";
  LandmarkGrid grid;
  for (const auto& pose : poses) {
    const int ccx = static_cast<int>(round(pose.truePose.translation().x()));
    const int ccy = static_cast<int>(round(pose.truePose.translation().y()));
    for (int a = -landmarksRange; a <= landmarksRange; a++)
      for (int b = -landmarksRange; b <= landmarksRange; b++) {
        const int cx = ccx + a;
        const int cy = ccy + b;
        LandmarkPtrVector& landmarksForCell = grid[cx][cy];
        if (landmarksForCell.empty()) {
          for (int i = 0; i < landMarksPerSquareMeter; ++i) {
            auto* l = new Landmark();
            double offx;
            double offy;
            do {
              offx = Sampler::uniformRand(-0.5 * stepLen, 0.5 * stepLen);
              offy = Sampler::uniformRand(-0.5 * stepLen, 0.5 * stepLen);
            } while (hypot_sqr(offx, offy) < 0.25 * 0.25);
            l->truePose[0] = cx + offx;
            l->truePose[1] = cy + offy;
            landmarksForCell.push_back(l);
          }
        }
      }
  }
  cerr << "done." << endl;

  cerr << "Simulator: Simulating landmark observations for the poses ... ";
  const double maxSensorSqr = maxSensorRangeLandmarks * maxSensorRangeLandmarks;
  int globalId = 0;
  for (auto& pose : poses) {
    Simulator::GridPose& pv = pose;
    const int cx = static_cast<int>(round(pose.truePose.translation().x()));
    const int cy = static_cast<int>(round(pose.truePose.translation().y()));
    const int numGridCells = static_cast<int>(maxSensorRangeLandmarks) + 1;

    pv.id = globalId++;
    const SE2 trueInv = pv.truePose.inverse();

    for (int xx = cx - numGridCells; xx <= cx + numGridCells; ++xx)
      for (int yy = cy - numGridCells; yy <= cy + numGridCells; ++yy) {
        const LandmarkPtrVector& landmarksForCell = grid[xx][yy];
        if (landmarksForCell.empty()) continue;
        for (auto* l : landmarksForCell) {
          const double dSqr =
              hypot_sqr(pv.truePose.translation().x() - l->truePose.x(),
                        pv.truePose.translation().y() - l->truePose.y());
          if (dSqr > maxSensorSqr) continue;
          const double obs = Sampler::uniformRand(0.0, 1.0);
          if (obs > observationProb)  // we do not see this one...
            continue;
          if (l->id < 0) l->id = globalId++;
          if (l->seenBy.empty()) {
            const Eigen::Vector2d trueObservation = trueInv * l->truePose;
            Eigen::Vector2d observation = trueObservation;
            observation[0] += Sampler::gaussRand(0., landmarkNoise[0]);
            observation[1] += Sampler::gaussRand(0., landmarkNoise[1]);
            l->simulatedPose = pv.simulatorPose * observation;
          }
          l->seenBy.push_back(pv.id);
          pv.landmarks.push_back(l);
        }
      }
  }
  cerr << "done." << endl;

  // add the odometry measurements
  odometry_.clear();
  cerr << "Simulator: Adding odometry measurements ... ";
  for (size_t i = 1; i < poses.size(); ++i) {
    const GridPose& prev = poses[i - 1];
    const GridPose& p = poses[i];

    odometry_.push_back(GridEdge());
    GridEdge& edge = odometry_.back();

    edge.from = prev.id;
    edge.to = p.id;
    edge.trueTransf = prev.truePose.inverse() * p.truePose;
    edge.simulatorTransf = prev.simulatorPose.inverse() * p.simulatorPose;
    edge.information = information;
  }
  cerr << "done." << endl;

  landmarks_.clear();
  landmarkObservations_.clear();
  // add the landmark observations
  {
    cerr << "Simulator: add landmark observations ... ";
    const Eigen::Matrix2d covariance =
        Eigen::Vector2d(landmarkNoise[0] * landmarkNoise[0],
                        landmarkNoise[1] * landmarkNoise[1])
            .asDiagonal();
    const Eigen::Matrix2d information = covariance.inverse();

    for (auto& p : poses) {
      for (size_t j = 0; j < p.landmarks.size(); ++j) {
        Landmark* l = p.landmarks[j];
        if (!l->seenBy.empty() && l->seenBy[0] == p.id) {
          landmarks.push_back(*l);
        }
      }
    }

    for (auto& p : poses) {
      const SE2 trueInv = (p.truePose * sensorOffset).inverse();
      for (size_t j = 0; j < p.landmarks.size(); ++j) {
        Landmark* l = p.landmarks[j];
        const Eigen::Vector2d trueObservation = trueInv * l->truePose;
        Eigen::Vector2d observation = trueObservation;
        if (!l->seenBy.empty() &&
            l->seenBy[0] ==
                p.id) {  // write the initial position of the landmark
          observation =
              (p.simulatorPose * sensorOffset).inverse() * l->simulatedPose;
        } else {
          // create observation for the LANDMARK using the true positions
          observation[0] += Sampler::gaussRand(0., landmarkNoise[0]);
          observation[1] += Sampler::gaussRand(0., landmarkNoise[1]);
        }

        landmarkObservations_.push_back(LandmarkEdge());
        LandmarkEdge& le = landmarkObservations_.back();

        le.from = p.id;
        le.to = l->id;
        le.trueMeas = trueObservation;
        le.simulatorMeas = observation;
        le.information = information;
      }
    }
    cerr << "done." << endl;
  }

  // cleaning up
  for (auto& it : grid) {
    for (auto& itt : it.second) {
      const Simulator::LandmarkPtrVector& landmarks = itt.second;
      for (auto& landmark : landmarks) delete landmark;
    }
  }
}

Simulator::GridPose Simulator::generateNewPose(
    const Simulator::GridPose& prev, const SE2& trueMotion,
    const Eigen::Vector2d& transNoise, double rotNoise) {
  Simulator::GridPose nextPose;
  nextPose.id = prev.id + 1;
  nextPose.truePose = prev.truePose * trueMotion;
  const SE2 noiseMotion =
      sampleTransformation(trueMotion, transNoise, rotNoise);
  nextPose.simulatorPose = prev.simulatorPose * noiseMotion;
  return nextPose;
}

SE2 Simulator::getMotion(int motionDirection, double stepLen) {
  switch (motionDirection) {
    case kMoLeft:
      return SE2(stepLen, 0, 0.5 * M_PI);
    case kMoRight:
      return SE2(stepLen, 0, -0.5 * M_PI);
    default:
      cerr << "Unknown motion direction" << endl;
      return SE2(stepLen, 0, -0.5 * M_PI);
  }
}

SE2 Simulator::sampleTransformation(const SE2& trueMotion_,
                                    const Eigen::Vector2d& transNoise,
                                    double rotNoise) {
  Eigen::Vector3d trueMotion = trueMotion_.toVector();
  SE2 noiseMotion(trueMotion[0] + Sampler::gaussRand(0.0, transNoise[0]),
                  trueMotion[1] + Sampler::gaussRand(0.0, transNoise[1]),
                  trueMotion[2] + Sampler::gaussRand(0.0, rotNoise));
  return noiseMotion;
}

}  // namespace tutorial
}  // namespace g2o
