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
#include "simulator2d.h"
#include "g2o/core/optimizable_graph.h"
#include <iostream>
#include <fstream>
#include <sstream>

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
  bool hasPointBearingSensor;
  bool hasCompass;
  bool hasGPS;

  bool hasSegmentSensor;
  int nSegments;
  int segmentGridSize;
  double minSegmentLenght, maxSegmentLenght;


  std::string outputFilename;
  arg.param("nlandmarks", nlandmarks, 100, "number of landmarks in the map");
  arg.param("nSegments", nSegments, 1000, "number of segments");
  arg.param("segmentGridSize", segmentGridSize, 50, "number of cells of the grid where to align the segments");
  arg.param("minSegmentLenght", minSegmentLenght, 0.5, "minimal lenght of a segment in the world");
  arg.param("maxSegmentLenght", maxSegmentLenght, 3,  "maximal lenght of a segment in the world");

  arg.param("simSteps", simSteps, 100, "number of simulation steps");
  arg.param("worldSize", worldSize, 25.0, "size of the world");
  arg.param("hasOdom",        hasOdom, false,  "the robot has an odometry" );
  arg.param("hasPointSensor", hasPointSensor, false, "the robot has a point sensor" );
  arg.param("hasPointBearingSensor", hasPointBearingSensor, false, "the robot has a point bearing sensor" );
  arg.param("hasPoseSensor",  hasPoseSensor, false,  "the robot has a pose sensor" );
  arg.param("hasCompass",     hasCompass, false, "the robot has a compass");
  arg.param("hasGPS",         hasGPS, false, "the robot has a GPS");
  arg.param("hasSegmentSensor", hasSegmentSensor, false, "the robot has a segment sensor" );
  arg.paramLeftOver("graph-output", outputFilename, "simulator_out.g2o", "graph file which will be written", true);


  arg.parseArgs(argc, argv);

  std::mt19937 generator;
  OptimizableGraph graph;
  World world(&graph);
  for (int i=0; i<nlandmarks; i++){
    WorldObjectPointXY * landmark = new WorldObjectPointXY;
    double x = sampleUniform(-.5,.5, &generator)*(worldSize+5);
    double y = sampleUniform(-.5,.5, &generator)*(worldSize+5);
    landmark->vertex()->setEstimate(Vector2d(x,y));
    world.addWorldObject(landmark);
  }

  cerr << "nSegments = " << nSegments << endl;

  for (int i=0; i<nSegments; i++){
    WorldObjectSegment2D * segment = new WorldObjectSegment2D;
    int ix = sampleUniform(-segmentGridSize,segmentGridSize, &generator);
    int iy = sampleUniform(-segmentGridSize,segmentGridSize, &generator);
    int ith = sampleUniform(0,3, &generator);
    double th= (M_PI/2)*ith;
    th=atan2(sin(th),cos(th));
    double xc = ix*(worldSize/segmentGridSize);
    double yc = iy*(worldSize/segmentGridSize);

    double l2 = sampleUniform(minSegmentLenght, maxSegmentLenght, &generator);

    double x1 = xc + cos(th)*l2;
    double y1 = yc + sin(th)*l2;
    double x2 = xc - cos(th)*l2;
    double y2 = yc - sin(th)*l2;


    segment->vertex()->setEstimateP1(Vector2d(x1,y1));
    segment->vertex()->setEstimateP2(Vector2d(x2,y2));
    world.addWorldObject(segment);
  }





  Robot2D robot(&world, "myRobot");
  world.addRobot(&robot);


  stringstream ss;
  ss << "-ws" << worldSize;
  ss << "-nl" << nlandmarks;
  ss << "-steps" << simSteps;

  if (hasOdom) {
    SensorOdometry2D* odometrySensor=new SensorOdometry2D("odometry");
    robot.addSensor(odometrySensor);
    Matrix3d odomInfo = odometrySensor->information();
    odomInfo.setIdentity();
    odomInfo*=500;
    odomInfo(2,2)=5000;
    odometrySensor->setInformation(odomInfo);
    ss << "-odom";
  }

  if (hasPoseSensor) {
    SensorPose2D* poseSensor = new SensorPose2D("poseSensor");
    robot.addSensor(poseSensor);
    Matrix3d poseInfo = poseSensor->information();
    poseInfo.setIdentity();
    poseInfo*=500;
    poseInfo(2,2)=5000;
    poseSensor->setInformation(poseInfo);
    ss << "-pose";
  }

  if (hasPointSensor) {
    SensorPointXY* pointSensor = new SensorPointXY("pointSensor");
    robot.addSensor(pointSensor);
    Matrix2d pointInfo = pointSensor->information();
    pointInfo.setIdentity();
    pointInfo*=1000;
    pointSensor->setInformation(pointInfo);
    pointSensor->setFov(0.75*M_PI);
    ss << "-pointXY";
  }

  if (hasPointBearingSensor) {
    SensorPointXYBearing* bearingSensor = new SensorPointXYBearing("bearingSensor");
    robot.addSensor(bearingSensor);
    bearingSensor->setInformation(bearingSensor->information()*1000);
    ss << "-pointBearing";
  }

  if (hasSegmentSensor) {
    cerr << "creating Segment Sensor" << endl;
    SensorSegment2D* segmentSensor = new SensorSegment2D("segmentSensor");
    cerr << "segmentSensorCreated" << endl;
    segmentSensor->setMaxRange(3);
    segmentSensor->setMinRange(.1);
    robot.addSensor(segmentSensor);
    segmentSensor->setInformation(segmentSensor->information()*1000);

    SensorSegment2DLine* segmentSensorLine = new SensorSegment2DLine("segmentSensorSensorLine");
    segmentSensorLine->setMaxRange(3);
    segmentSensorLine->setMinRange(.1);
    robot.addSensor(segmentSensorLine);
    Matrix2d m=segmentSensorLine->information();
    m=m*1000;
    m(0,0)*=10;
    segmentSensorLine->setInformation(m);

    SensorSegment2DPointLine* segmentSensorPointLine = new SensorSegment2DPointLine("segmentSensorSensorPointLine");
    segmentSensorPointLine->setMaxRange(3);
    segmentSensorPointLine->setMinRange(.1);
    robot.addSensor(segmentSensorPointLine);
    Matrix3d m3=segmentSensorPointLine->information();
    m3=m3*1000;
    m3(3,3)*=10;
    segmentSensorPointLine->setInformation(m3);

    ss << "-segment2d";
  }


  robot.move(SE2());
  double pStraight=0.7;
  SE2 moveStraight; moveStraight.setTranslation(Vector2d(1., 0.));
  double pLeft=0.15;
  SE2 moveLeft; moveLeft.setRotation(Rotation2Dd(M_PI/2));
  //double pRight=0.15;
  SE2 moveRight; moveRight.setRotation(Rotation2Dd(-M_PI/2));

  for (int i=0; i<simSteps; i++){
    cerr << "m";
    SE2 move;
    SE2 pose=robot.pose();
    double dtheta=-100;
    Vector2d dt;
    if (pose.translation().x() < -.5*worldSize){
      dtheta = 0;
    }

    if (pose.translation().x() >  .5*worldSize){
      dtheta = -M_PI;
    }

    if (pose.translation().y() < -.5*worldSize){
      dtheta = M_PI/2;
    }

    if (pose.translation().y() >  .5*worldSize){
      dtheta = -M_PI/2;
    }
    if (dtheta< -M_PI) {
      // select a random move of the robot
      double sampled=sampleUniform(0.,1.,&generator);
      if (sampled<pStraight)
        move=moveStraight;
      else if (sampled<pStraight+pLeft)
        move=moveLeft;
      else
        move=moveRight;
    } else {
      double mTheta=dtheta-pose.rotation().angle();
      move.setRotation(Rotation2Dd(mTheta));
      if (move.rotation().angle()<std::numeric_limits<double>::epsilon()){
        move.setTranslation(Vector2d(1., 0.));
      }
    }
    robot.relativeMove(move);
    // do a sense
    cerr << "s";
    robot.sense();
  }
  //string fname=outputFilename + ss.str() + ".g2o";
  ofstream testStream(outputFilename.c_str());
  graph.save(testStream);

  return 0;
}
