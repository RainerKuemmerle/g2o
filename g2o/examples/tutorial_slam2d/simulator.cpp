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

#include "rand.h"

#include <map>
#include <iostream>
#include <cmath>
using namespace std;

namespace g2o {
  namespace tutorial {

    using namespace Eigen;

#  ifdef _MSC_VER
    inline double round(double number)
    {
      return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
    }
#  endif

    typedef std::map<int, std::map<int, Simulator::LandmarkPtrVector> > LandmarkGrid;

    Simulator::Simulator()
    {
      time_t seed = time(0);
      Rand::seed_rand(static_cast<unsigned int>(seed));
    }

    Simulator::~Simulator()
    {
    }

    void Simulator::simulate(int numNodes, const SE2& sensorOffset)
    {
      // simulate a robot observing landmarks while travelling on a grid
      int steps = 5;
      double stepLen = 1.0;
      int boundArea = 50;

      double maxSensorRangeLandmarks = 2.5 * stepLen;

      int landMarksPerSquareMeter = 1;
      double observationProb = 0.8;

      int landmarksRange=2;

      Vector2d transNoise(0.05, 0.01);
      double rotNoise = DEG2RAD(2.);
      Vector2d landmarkNoise(0.05, 0.05);

      Vector2d bound(boundArea, boundArea);

      VectorXd probLimits;
      probLimits.resize(MO_NUM_ELEMS);
      for (int i = 0; i < probLimits.size(); ++i)
        probLimits[i] = (i + 1) / (double) MO_NUM_ELEMS;

      Matrix3d covariance;
      covariance.fill(0.);
      covariance(0, 0) = transNoise[0]*transNoise[0];
      covariance(1, 1) = transNoise[1]*transNoise[1];
      covariance(2, 2) = rotNoise*rotNoise;
      Matrix3d information = covariance.inverse();

      SE2 maxStepTransf(stepLen * steps, 0, 0);
      Simulator::PosesVector& poses = _poses;
      poses.clear();
      LandmarkVector& landmarks = _landmarks;
      landmarks.clear();
      Simulator::GridPose firstPose;
      firstPose.id = 0;
      firstPose.truePose = SE2(0,0,0);
      firstPose.simulatorPose = SE2(0,0,0);
      poses.push_back(firstPose);
      cerr << "Simulator: sampling nodes ...";
      while ((int)poses.size() < numNodes) {
        // add straight motions
        for (int i = 1; i < steps && (int)poses.size() < numNodes; ++i) {
          Simulator::GridPose nextGridPose = generateNewPose(poses.back(), SE2(stepLen,0,0), transNoise, rotNoise);
          poses.push_back(nextGridPose);
        }
        if ((int)poses.size() == numNodes)
          break;

        // sample a new motion direction
        double sampleMove = Rand::uniform_rand(0., 1.);
        int motionDirection = 0;
        while (probLimits[motionDirection] < sampleMove && motionDirection+1 < MO_NUM_ELEMS) {
          motionDirection++;
        }

        SE2 nextMotionStep = getMotion(motionDirection, stepLen);
        Simulator::GridPose nextGridPose = generateNewPose(poses.back(), nextMotionStep, transNoise, rotNoise);

        // check whether we will walk outside the boundaries in the next iteration
        SE2 nextStepFinalPose = nextGridPose.truePose * maxStepTransf;
        if (fabs(nextStepFinalPose.translation().x()) >= bound[0] || fabs(nextStepFinalPose.translation().y()) >= bound[1]) {
          //cerr << "b";
          // will be outside boundaries using this
          for (int i = 0; i < MO_NUM_ELEMS; ++i) {
            nextMotionStep = getMotion(i, stepLen);
            nextGridPose = generateNewPose(poses.back(), nextMotionStep, transNoise, rotNoise);
            nextStepFinalPose = nextGridPose.truePose * maxStepTransf;
            if (fabs(nextStepFinalPose.translation().x()) < bound[0] && fabs(nextStepFinalPose.translation().y()) < bound[1])
              break;
          }
        }

        poses.push_back(nextGridPose);
      }
      cerr << "done." << endl;

      // creating landmarks along the trajectory
      cerr << "Simulator: Creating landmarks ... ";
      LandmarkGrid grid;
      for (PosesVector::const_iterator it = poses.begin(); it != poses.end(); ++it) {
        int ccx = (int)round(it->truePose.translation().x());
        int ccy = (int)round(it->truePose.translation().y());
        for (int a=-landmarksRange; a<=landmarksRange; a++)
          for (int b=-landmarksRange; b<=landmarksRange; b++){
            int cx=ccx+a;
            int cy=ccy+b;
            LandmarkPtrVector& landmarksForCell = grid[cx][cy];
            if (landmarksForCell.size() == 0) {
              for (int i = 0; i < landMarksPerSquareMeter; ++i) {
                Landmark* l = new Landmark();
                double offx, offy;
                do {
                  offx = Rand::uniform_rand(-0.5*stepLen, 0.5*stepLen);
                  offy = Rand::uniform_rand(-0.5*stepLen, 0.5*stepLen);
                } while (hypot_sqr(offx, offy) < 0.25*0.25);
                l->truePose[0] = cx + offx;
                l->truePose[1] = cy + offy;
                landmarksForCell.push_back(l);
              }
            }
          }
      }
      cerr << "done." << endl;

      cerr << "Simulator: Simulating landmark observations for the poses ... ";
      double maxSensorSqr = maxSensorRangeLandmarks * maxSensorRangeLandmarks;
      int globalId = 0;
      for (PosesVector::iterator it = poses.begin(); it != poses.end(); ++it) {
        Simulator::GridPose& pv = *it;
        int cx = (int)round(it->truePose.translation().x());
        int cy = (int)round(it->truePose.translation().y());
        int numGridCells = (int)(maxSensorRangeLandmarks) + 1;

        pv.id = globalId++;
        SE2 trueInv = pv.truePose.inverse();

        for (int xx = cx - numGridCells; xx <= cx + numGridCells; ++xx)
          for (int yy = cy - numGridCells; yy <= cy + numGridCells; ++yy) {
            LandmarkPtrVector& landmarksForCell = grid[xx][yy];
            if (landmarksForCell.size() == 0)
              continue;
            for (size_t i = 0; i < landmarksForCell.size(); ++i) {
              Landmark* l = landmarksForCell[i];
              double dSqr = hypot_sqr(pv.truePose.translation().x() - l->truePose.x(), pv.truePose.translation().y() - l->truePose.y());
              if (dSqr > maxSensorSqr)
                continue;
              double obs = Rand::uniform_rand(0.0, 1.0);
              if (obs > observationProb) // we do not see this one...
                continue;
              if (l->id < 0)
                l->id = globalId++;
              if (l->seenBy.size() == 0) {
                Vector2d trueObservation = trueInv * l->truePose;
                Vector2d observation = trueObservation;
                observation[0] += Rand::gauss_rand(0., landmarkNoise[0]);
                observation[1] += Rand::gauss_rand(0., landmarkNoise[1]);
                l->simulatedPose = pv.simulatorPose * observation;
              }
              l->seenBy.push_back(pv.id);
              pv.landmarks.push_back(l);
            }
          }

      }
      cerr << "done." << endl;

      // add the odometry measurements
      _odometry.clear();
      cerr << "Simulator: Adding odometry measurements ... ";
      for (size_t i = 1; i < poses.size(); ++i) {
        const GridPose& prev = poses[i-1];
        const GridPose& p = poses[i];

        _odometry.push_back(GridEdge());
        GridEdge& edge = _odometry.back();

        edge.from = prev.id;
        edge.to = p.id;
        edge.trueTransf = prev.truePose.inverse() * p.truePose;
        edge.simulatorTransf = prev.simulatorPose.inverse() * p.simulatorPose;
        edge.information = information;
      }
      cerr << "done." << endl;

      _landmarks.clear();
      _landmarkObservations.clear();
      // add the landmark observations
      {
        cerr << "Simulator: add landmark observations ... ";
        Matrix2d covariance; covariance.fill(0.);
        covariance(0, 0) = landmarkNoise[0]*landmarkNoise[0];
        covariance(1, 1) = landmarkNoise[1]*landmarkNoise[1];
        Matrix2d information = covariance.inverse();

        for (size_t i = 0; i < poses.size(); ++i) {
          const GridPose& p = poses[i];
          for (size_t j = 0; j < p.landmarks.size(); ++j) {
            Landmark* l = p.landmarks[j];
            if (l->seenBy.size() > 0 && l->seenBy[0] == p.id) {
              landmarks.push_back(*l);
            }
          }
        }

        for (size_t i = 0; i < poses.size(); ++i) {
          const GridPose& p = poses[i];
          SE2 trueInv = (p.truePose * sensorOffset).inverse();
          for (size_t j = 0; j < p.landmarks.size(); ++j) {
            Landmark* l = p.landmarks[j];
            Vector2d observation;
            Vector2d trueObservation = trueInv * l->truePose;
            observation = trueObservation;
            if (l->seenBy.size() > 0 && l->seenBy[0] == p.id) { // write the initial position of the landmark
              observation = (p.simulatorPose * sensorOffset).inverse() * l->simulatedPose;
            } else {
              // create observation for the LANDMARK using the true positions
              observation[0] += Rand::gauss_rand(0., landmarkNoise[0]);
              observation[1] += Rand::gauss_rand(0., landmarkNoise[1]);
            }

            _landmarkObservations.push_back(LandmarkEdge());
            LandmarkEdge& le = _landmarkObservations.back();

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
      for (LandmarkGrid::iterator it = grid.begin(); it != grid.end(); ++it) {
        for (std::map<int, Simulator::LandmarkPtrVector>::iterator itt = it->second.begin(); itt != it->second.end(); ++itt) {
          Simulator::LandmarkPtrVector& landmarks = itt->second;
          for (size_t i = 0; i < landmarks.size(); ++i)
            delete landmarks[i];
        }
      }

    }

    Simulator::GridPose Simulator::generateNewPose(const Simulator::GridPose& prev, const SE2& trueMotion, const Eigen::Vector2d& transNoise, double rotNoise)
    {
      Simulator::GridPose nextPose;
      nextPose.id = prev.id + 1;
      nextPose.truePose = prev.truePose * trueMotion;
      SE2 noiseMotion = sampleTransformation(trueMotion, transNoise, rotNoise);
      nextPose.simulatorPose = prev.simulatorPose * noiseMotion;
      return nextPose;
    }

    SE2 Simulator::getMotion(int motionDirection, double stepLen)
    {
      switch (motionDirection) {
        case MO_LEFT:
          return SE2(stepLen, 0, 0.5*M_PI);
        case MO_RIGHT:
          return SE2(stepLen, 0, -0.5*M_PI);
        default:
          cerr << "Unknown motion direction" << endl;
          return SE2(stepLen, 0, -0.5*M_PI);
      }
    }

    SE2 Simulator::sampleTransformation(const SE2& trueMotion_, const Eigen::Vector2d& transNoise, double rotNoise)
    {
      Vector3d trueMotion = trueMotion_.toVector();
      SE2 noiseMotion(
          trueMotion[0] + Rand::gauss_rand(0.0, transNoise[0]),
          trueMotion[1] + Rand::gauss_rand(0.0, transNoise[1]),
          trueMotion[2] + Rand::gauss_rand(0.0, rotNoise));
      return noiseMotion;
    }

  } // end namespace
} // end namespace
