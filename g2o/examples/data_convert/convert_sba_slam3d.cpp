#include <iostream>
#include <fstream>

#include "g2o/stuff/macros.h"
#include "g2o/stuff/command_args.h"

#include "g2o/core/optimizable_graph.h"

#include "g2o/types/sba/types_sba.h"
#include "g2o/types/slam3d/parameter_camera.h"
#include "g2o/types/slam3d/edge_se3_pointxyz_disparity.h"

using namespace std;
using namespace g2o;

int main(int argc, char** argv)
{
  string inputFilename;
  string outputFilename;
  // command line parsing
  CommandArgs commandLineArguments;
  commandLineArguments.paramLeftOver("gm2dl-input", inputFilename, "", "gm2dl file which will be processed");
  commandLineArguments.paramLeftOver("gm2dl-output", outputFilename, "", "name of the output file");

  commandLineArguments.parseArgs(argc, argv);

  OptimizableGraph inputGraph;
  bool loadStatus = inputGraph.load(inputFilename.c_str());
  if (! loadStatus) {
    cerr << "Error while loading input data" << endl;
    return 1;
  }

  OptimizableGraph outputGraph;

  // process all the vertices first
  double fx = -1;
  double baseline = -1;
  bool firstCam = true;
  for (OptimizableGraph::VertexIDMap::const_iterator it = inputGraph.vertices().begin(); it != inputGraph.vertices().end(); ++it) {
    if (dynamic_cast<VertexCam*>(it->second)) {
      VertexCam* v = static_cast<VertexCam*>(it->second);
      if (firstCam) {
        firstCam = false;
        g2o::ParameterCamera* camParams = new g2o::ParameterCamera;
        camParams->setId(0);
        const SBACam& c = v->estimate();
        baseline = c.baseline;
        fx = c.Kcam(0,0);
        camParams->setKcam(c.Kcam(0,0), c.Kcam(1,1), c.Kcam(0,2), c.Kcam(1,2));
        outputGraph.addParameter(camParams);
      }

      VertexSE3* ov = new VertexSE3;
      ov->setId(v->id());
      ov->setEstimate(v->estimate());
      if (! outputGraph.addVertex(ov)) {
        assert(0 && "Failure adding camera vertex");
      }
    }
    else if (dynamic_cast<VertexSBAPointXYZ*>(it->second)) {
      VertexSBAPointXYZ* v = static_cast<VertexSBAPointXYZ*>(it->second);

      VertexPointXYZ* ov = new VertexPointXYZ;
      ov->setId(v->id());
      ov->setEstimate(v->estimate());
      if (! outputGraph.addVertex(ov)) {
        assert(0 && "Failure adding camera vertex");
      }
    }
  }
  
  for (OptimizableGraph::EdgeSet::const_iterator it = inputGraph.edges().begin(); it != inputGraph.edges().end(); ++it) {
    if (dynamic_cast<EdgeProjectP2SC*>(*it)) {
      EdgeProjectP2SC* e = static_cast<EdgeProjectP2SC*>(*it);

      EdgeSE3PointXYZDisparity* oe = new EdgeSE3PointXYZDisparity;
      oe->vertices()[0] = outputGraph.vertex(e->vertices()[1]->id());
      oe->vertices()[1] = outputGraph.vertex(e->vertices()[0]->id());

      double kx = e->measurement().x();
      double ky = e->measurement().y();
      double disparity = kx - e->measurement()(2);

      oe->setMeasurement(Eigen::Vector3d(kx, ky, disparity / (fx * baseline)));
      oe->setInformation(e->information()); // TODO convert information matrix
      oe->setParameterId(0,0);
      if (! outputGraph.addEdge(oe)) {
        assert(0 && "error adding edge");
      }
    }
  }

  cout << "Vertices in/out:\t" << inputGraph.vertices().size() << " " << outputGraph.vertices().size() << endl;
  cout << "Edges in/out:\t" << inputGraph.edges().size() << " " << outputGraph.edges().size() << endl;

  cout << "Writing output ... " << flush;
  outputGraph.save(outputFilename.c_str());
  cout << "done." << endl;
  return 0;
}
