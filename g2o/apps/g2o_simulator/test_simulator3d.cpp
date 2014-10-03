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
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/sampler.h"
#include "simulator3d.h"
#include "g2o/core/optimizable_graph.h"
#include <iostream>
#include <fstream>

//#define _POSE_SENSOR_OFFSET
//#define _POSE_PRIOR_SENSOR

using namespace g2o;
using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  CommandArgs arg;
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
  arg.param("hasOdom",        hasOdom, false,  "the robot has an odometry" );
  arg.param("hasPointSensor", hasPointSensor, false, "the robot has a point sensor" );
  arg.param("hasPointDepthSensor", hasPointDepthSensor, false, "the robot has a point sensor" );
  arg.param("hasPointDisparitySensor", hasPointDisparitySensor, false, "the robot has a point sensor" );
  arg.param("hasPoseSensor",  hasPoseSensor, false,  "the robot has a pose sensor" );
  arg.param("hasCompass",     hasCompass, false, "the robot has a compass");
  arg.param("hasGPS",         hasGPS, false, "the robot has a GPS");
  arg.paramLeftOver("graph-output", outputFilename, "simulator_out.g2o", "graph file which will be written", true);
 
  arg.parseArgs(argc, argv);

  std::mt19937 generator;
  OptimizableGraph graph;
  World world(&graph);
  for (int i=0; i<nlandmarks; i++){
    WorldObjectTrackXYZ * landmark = new WorldObjectTrackXYZ;
    double x = sampleUniform(-.5, .5, &generator)*worldSize;
    double y = sampleUniform(-.5, .5, &generator)*worldSize;
    double z = sampleUniform(-.5, .5);
    landmark->vertex()->setEstimate(Vector3d(x,y,z));
    world.addWorldObject(landmark);
  }
  Robot3D robot(&world, "myRobot");
  world.addRobot(&robot);

  stringstream ss;
  ss << "-ws" << worldSize;
  ss << "-nl" << nlandmarks;
  ss << "-steps" << simSteps;

  if (hasOdom) {
    SensorOdometry3D* odometrySensor=new SensorOdometry3D("odometry");
    robot.addSensor(odometrySensor);
    ss << "-odom";
  }

  if (hasPointSensor) {
    SensorPointXYZ* pointSensor =  new SensorPointXYZ("pointSensor");
    pointSensor->setFov(M_PI/4);
    robot.addSensor(pointSensor);
    Eigen::Isometry3d cameraPose;
    Eigen::Matrix3d R;
    R  << 0,  0,  1,
         -1,  0,  0,
          0, -1,  0;
    pointSensor->setMaxRange(2.);
    cameraPose = R;
    cameraPose.translation() = Vector3d(0.,0.,0.3);
    pointSensor->offsetParam()->setOffset(cameraPose);
    ss << "-pointXYZ";
  }
  
  if (hasPointDisparitySensor){
    SensorPointXYZDisparity* disparitySensor = new SensorPointXYZDisparity("disparitySensor");
    disparitySensor->setFov(M_PI/4);
    disparitySensor->setMinRange(0.5);
    disparitySensor->setMaxRange(2.);
    robot.addSensor(disparitySensor);
    Eigen::Isometry3d cameraPose;
    Eigen::Matrix3d R;
    R  << 0,  0,  1,
         -1,  0,  0, 
          0, -1,  0;
    cameraPose = R;
    cameraPose.translation() = Vector3d(0.,0.,0.3);
    disparitySensor->offsetParam()->setOffset(cameraPose);
    ss << "-disparity";
  }

  if (hasPointDepthSensor){
    SensorPointXYZDepth* depthSensor = new SensorPointXYZDepth("depthSensor");
    depthSensor->setFov(M_PI/4);
    depthSensor->setMinRange(0.5);
    depthSensor->setMaxRange(2.);
    robot.addSensor(depthSensor);
    Eigen::Isometry3d cameraPose;
    Eigen::Matrix3d R;
    R  << 0,  0,  1,
         -1,  0,  0, 
          0, -1,  0;
    cameraPose = R;
    cameraPose.translation() = Vector3d(0.,0.,0.3);
    depthSensor->offsetParam()->setOffset(cameraPose);
    ss << "-depth";
  }

  if (hasPoseSensor){
    SensorPose3D* poseSensor = new SensorPose3D("poseSensor");
    robot.addSensor(poseSensor);
    poseSensor->setMaxRange(5);
    ss << "-pose";
  }

#ifdef _POSE_PRIOR_SENSOR  
  SensorSE3Prior posePriorSensor("posePriorSensor");
  robot.addSensor(&posePriorSensor);
  {
    Eigen::Isometry3d cameraPose;
    Eigen::Matrix3d R;
    R  << 0,  0,  1,
         -1,  0,  0,
          0, -1,  0;
    cameraPose = R;
    cameraPose.translation() = Vector3d(0.,0.,0.3);
    posePriorSensor.offsetParam()->setOffset(cameraPose);
  }
#endif

#ifdef _POSE_SENSOR_OFFSET
  SensorPose3DOffset poseSensor("poseSensor");
  poseSensor.setFov(M_PI/4);
  poseSensor.setMinRange(0.5);
  poseSensor.setMaxRange(5);
  robot.addSensor(&poseSensor);
  if(0){
    Eigen::Isometry3d cameraPose;
    Eigen::Matrix3d R;
    R  << 0,  0,  1,
         -1,  0,  0, 
          0, -1,  0;
    cameraPose = R;
    cameraPose.translation() = Vector3d(0.,0.,0.3);
    poseSensor.offsetParam1()->setOffset(cameraPose);
    poseSensor.offsetParam2()->setOffset(cameraPose);
  }
#endif  


  robot.move(Eigen::Isometry3d::Identity());
  double pStraight=0.7;
  Eigen::Isometry3d moveStraight = Eigen::Isometry3d::Identity(); moveStraight.translation() = Vector3d(1., 0., 0.);
  double pLeft=0.15;
  Eigen::Isometry3d moveLeft = Eigen::Isometry3d::Identity(); moveLeft = AngleAxisd(M_PI/2, Vector3d::UnitZ());
  //double pRight=0.15;
  Eigen::Isometry3d moveRight = Eigen::Isometry3d::Identity(); moveRight = AngleAxisd(-M_PI/2,Vector3d::UnitZ());
  
  Eigen::Matrix3d dtheta = Eigen::Matrix3d::Identity();
  for (int i=0; i<simSteps; i++){
    bool boundariesReached = true;
    cerr << "m";
    Vector3d dt;
    const Eigen::Isometry3d& pose = robot.pose();
    if (pose.translation().x() < -.5*worldSize){
      dtheta = AngleAxisd(0,Vector3d::UnitZ());
    } else if (pose.translation().x() >  .5*worldSize){
      dtheta = AngleAxisd(-M_PI,Vector3d::UnitZ());
    } else if (pose.translation().y() < -.5*worldSize){
      dtheta = AngleAxisd(M_PI/2,Vector3d::UnitZ());
    } else if (pose.translation().y() >  .5*worldSize){
      dtheta = AngleAxisd(-M_PI/2,Vector3d::UnitZ());
    } else {
      boundariesReached=false;
    }
    
    Eigen::Isometry3d move = Eigen::Isometry3d::Identity();
    if (boundariesReached){
      Eigen::Matrix3d mTheta = pose.rotation().inverse() * dtheta;
      move = mTheta;
      AngleAxisd aa(mTheta);
      if (aa.angle()<std::numeric_limits<double>::epsilon()){
        move.translation() = Vector3d(1., 0., 0.);
      }
    } else {
      double sampled=sampleUniform();
      if (sampled<pStraight)
        move=moveStraight;
      else if (sampled<pStraight+pLeft)
        move=moveLeft;
      else
        move=moveRight;
    }
    

    // select a random move of the robot
    robot.relativeMove(move);
    // do a sense
    cerr << "s";
    robot.sense();
  }
  string fname=outputFilename + ss.str() + ".g2o";
  //ofstream testStream(fname.c_str());
  ofstream testStream(outputFilename.c_str());
  graph.save(testStream);
 
}
