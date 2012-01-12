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

  std::tr1::ranlux_base_01 generator;
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
    SE3Quat cameraPose;
    Eigen::Matrix3d R;
    R  << 0,  0,  1,
         -1,  0,  0,
          0, -1,  0;
    pointSensor->setMaxRange(2.);
    cameraPose.setRotation(Quaterniond(R));
    cameraPose.setTranslation(Vector3d(0.,0.,0.3));
    pointSensor->offsetParam()->setOffset(cameraPose);
    ss << "-pointXYZ";
  }
  
  if (hasPointDisparitySensor){
    SensorPointXYZDisparity* disparitySensor = new SensorPointXYZDisparity("disparitySensor");
    disparitySensor->setFov(M_PI/4);
    disparitySensor->setMinRange(0.5);
    disparitySensor->setMaxRange(2.);
    robot.addSensor(disparitySensor);
    SE3Quat cameraPose;
    Eigen::Matrix3d R;
    R  << 0,  0,  1,
         -1,  0,  0, 
          0, -1,  0;
    cameraPose.setRotation(Quaterniond(R));
    cameraPose.setTranslation(Vector3d(0.,0.,0.3));
    disparitySensor->offsetParam()->setOffset(cameraPose);
    ss << "-disparity";
  }

  if (hasPointDepthSensor){
    SensorPointXYZDepth* depthSensor = new SensorPointXYZDepth("depthSensor");
    depthSensor->setFov(M_PI/4);
    depthSensor->setMinRange(0.5);
    depthSensor->setMaxRange(2.);
    robot.addSensor(depthSensor);
    SE3Quat cameraPose;
    Eigen::Matrix3d R;
    R  << 0,  0,  1,
         -1,  0,  0, 
          0, -1,  0;
    cameraPose.setRotation(Quaterniond(R));
    cameraPose.setTranslation(Vector3d(0.,0.,0.3));
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
    SE3Quat cameraPose;
    Eigen::Matrix3d R;
    R  << 0,  0,  1,
         -1,  0,  0,
          0, -1,  0;
    cameraPose.setRotation(Quaterniond(R));
    cameraPose.setTranslation(Vector3d(0.,0.,0.3));
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
    SE3Quat cameraPose;
    Eigen::Matrix3d R;
    R  << 0,  0,  1,
         -1,  0,  0, 
          0, -1,  0;
    cameraPose.setRotation(Quaterniond(R));
    cameraPose.setTranslation(Vector3d(0.,0.,0.3));
    poseSensor.offsetParam1()->setOffset(cameraPose);
    poseSensor.offsetParam2()->setOffset(cameraPose);
  }
#endif  


  robot.move(SE3Quat());
  double pStraight=0.7;
  SE3Quat moveStraight; moveStraight.setTranslation(Vector3d(1., 0., 0.));
  double pLeft=0.15;
  SE3Quat moveLeft; moveLeft.setRotation(Quaterniond(AngleAxisd(M_PI/2,Vector3d::UnitZ())));
  //double pRight=0.15;
  SE3Quat moveRight; moveRight.setRotation(Quaterniond(AngleAxisd(-M_PI/2,Vector3d::UnitZ())));
  
  Quaterniond dtheta(1.,0.,0.,0.);
  for (int i=0; i<simSteps; i++){
    bool boundariesReached = true;
    cerr << "m";
    Vector3d dt;
    SE3Quat pose=robot.pose();
    if (pose.translation().x() < -.5*worldSize){
      dtheta = Quaterniond(AngleAxisd(0,Vector3d::UnitZ()));
    } else if (pose.translation().x() >  .5*worldSize){
      dtheta = Quaterniond(AngleAxisd(-M_PI,Vector3d::UnitZ()));
    } else if (pose.translation().y() < -.5*worldSize){
      dtheta = Quaterniond(AngleAxisd(M_PI/2,Vector3d::UnitZ()));
    } else if (pose.translation().y() >  .5*worldSize){
      dtheta = Quaterniond(AngleAxisd(-M_PI/2,Vector3d::UnitZ()));
    } else {
      boundariesReached=false;
    }
    
    SE3Quat move;
    if (boundariesReached){
      Quaterniond mTheta=pose.rotation().inverse()*dtheta;
      move.setRotation(mTheta);
      AngleAxisd aa(mTheta.toRotationMatrix());
      if (aa.angle()<std::numeric_limits<double>::epsilon()){
        move.setTranslation(Vector3d(1., 0., 0.));
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
