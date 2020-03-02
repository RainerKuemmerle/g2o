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

#ifndef G2O_SIMULATOR_H
#define G2O_SIMULATOR_H

#include "se2.h"
#include "g2o_tutorial_slam2d_api.h"

#include <vector>
#include <map>

namespace g2o {
  namespace tutorial {

    class G2O_TUTORIAL_SLAM2D_API Simulator {
      public:

        enum G2O_TUTORIAL_SLAM2D_API MotionType {
          MO_LEFT, MO_RIGHT,
          MO_NUM_ELEMS
        };

        /**
         * \brief simulated landmark
         */
        struct G2O_TUTORIAL_SLAM2D_API Landmark
        {
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
          int id;
          Eigen::Vector2d truePose;
          Eigen::Vector2d simulatedPose;
          std::vector<int> seenBy;
          Landmark() : id(-1) {}
        };
        typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark> > LandmarkVector;
        typedef std::vector<Landmark*> LandmarkPtrVector;

        /**
         * simulated pose of the robot
         */
        struct G2O_TUTORIAL_SLAM2D_API GridPose
        {
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
          int id;
          SE2 truePose;
          SE2 simulatorPose;
          LandmarkPtrVector landmarks;      ///< the landmarks observed by this node
        };
        typedef std::vector<GridPose, Eigen::aligned_allocator<GridPose> >  PosesVector;

        /**
         * \brief odometry constraint
         */
        struct G2O_TUTORIAL_SLAM2D_API GridEdge
        {
          int from;
          int to;
          SE2 trueTransf;
          SE2 simulatorTransf;
          Eigen::Matrix3d information;
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        };
        typedef std::vector<GridEdge, Eigen::aligned_allocator<GridEdge> >  GridEdgeVector;

        struct G2O_TUTORIAL_SLAM2D_API LandmarkEdge
        {
          int from;
          int to;
          Eigen::Vector2d trueMeas;
          Eigen::Vector2d simulatorMeas;
          Eigen::Matrix2d information;
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        };
        typedef std::vector<LandmarkEdge, Eigen::aligned_allocator<LandmarkEdge> >  LandmarkEdgeVector;

      public:
        Simulator();
        ~Simulator();

        void simulate(int numPoses, const SE2& sensorOffset = SE2());

        const PosesVector& poses() const { return _poses;}
        const LandmarkVector& landmarks() const { return _landmarks;}
        const GridEdgeVector& odometry() const { return _odometry;}
        const LandmarkEdgeVector& landmarkObservations() const { return _landmarkObservations;}

      protected:
        PosesVector _poses;
        LandmarkVector _landmarks;
        GridEdgeVector _odometry;
        LandmarkEdgeVector _landmarkObservations;

        GridPose generateNewPose(const GridPose& prev, const SE2& trueMotion, const Eigen::Vector2d& transNoise, double rotNoise);
        SE2 getMotion(int motionDirection, double stepLen);
        SE2 sampleTransformation(const SE2& trueMotion_, const Eigen::Vector2d& transNoise, double rotNoise);
    };

  } // end namespace
} // end namespace

#endif
