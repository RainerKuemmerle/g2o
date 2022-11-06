// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G.Grisetti, W. Burgard
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

#include "gm2dl_io.h"

#include <fstream>
#include <iostream>
#include <memory>

#include "g2o/core/factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/types/data/data_queue.h"
#include "g2o/types/data/robot_laser.h"
#include "g2o/types/sclam2d/edge_se2_sensor_calib.h"

namespace g2o {

const int Gm2dlIO::kIdLaserpose = std::numeric_limits<int>::max();
const int Gm2dlIO::kIdOdomcalib = std::numeric_limits<int>::max() - 1;

bool Gm2dlIO::readGm2dl(const std::string& filename, SparseOptimizer& optimizer,
                        bool overrideCovariances) {
  optimizer.clear();
  std::ifstream is(filename.c_str());
  if (!is.good()) return false;

  bool laserOffsetInitDone = false;
  auto laserOffset = std::make_shared<VertexSE2>();
  // laserOffset->fixed() = true;
  laserOffset->setId(kIdLaserpose);
  if (!optimizer.addVertex(laserOffset)) {
    std::cerr << "Unable to add laser offset" << std::endl;
    return false;
  }

  // parse the GM2DL file an extract the vertices, edges, and the laser data
  std::stringstream currentLine;
  std::shared_ptr<VertexSE2> previousVertex;
  while (true) {
    const int bytesRead = readLine(is, currentLine);
    if (bytesRead == -1) break;
    std::string tag;
    currentLine >> tag;
    if (tag == "VERTEX" || tag == "VERTEX2" || tag == "VERTEX_SE2") {
      int id;
      Eigen::Vector3d p;
      currentLine >> id >> p.x() >> p.y() >> p.z();
      // adding the robot pose
      auto v = std::make_shared<VertexSE2>();
      v->setId(id);
      // std::cerr << "Read vertex id " << id << std::endl;
      if (!optimizer.addVertex(v)) {
        std::cerr << "vertex " << id << " is already in the graph, reassigning "
                  << std::endl;
        v = std::dynamic_pointer_cast<VertexSE2>(optimizer.vertex(id));
        assert(v);
      }
      v->setEstimate(SE2(p));
      previousVertex = v;

    } else if (tag == "EDGE" || tag == "EDGE2" || tag == "EDGE_SE2") {
      if (!laserOffsetInitDone) {
        std::cerr << "Error: need laser offset" << std::endl;
        return false;
      }
      int id1;
      int id2;
      auto e = std::make_shared<EdgeSE2SensorCalib>();
      Eigen::Vector3d p;
      Eigen::Matrix3d& m = e->information();
      currentLine >> id1 >> id2 >> p.x() >> p.y() >> p.z();
      if (overrideCovariances) {
        m = Eigen::Matrix3d::Identity();
      } else {
        if (tag == "EDGE_SE2")
          currentLine >> m(0, 0) >> m(0, 1) >> m(0, 2) >> m(1, 1) >> m(1, 2) >>
              m(2, 2);
        else  // old stupid order of the information matrix
          currentLine >> m(0, 0) >> m(0, 1) >> m(1, 1) >> m(2, 2) >> m(0, 2) >>
              m(1, 2);
        m(1, 0) = m(0, 1);
        m(2, 0) = m(0, 2);
        m(2, 1) = m(1, 2);
      }
      previousVertex = nullptr;
      auto v1 = std::dynamic_pointer_cast<VertexSE2>(optimizer.vertex(id1));
      auto v2 = std::dynamic_pointer_cast<VertexSE2>(optimizer.vertex(id2));
      if (!v1) {
        std::cerr << "vertex " << id1 << " is not existing, cannot add edge ("
                  << id1 << "," << id2 << ")" << std::endl;
        continue;
      }
      if (!v2) {
        std::cerr << "vertex " << id2 << " is not existing, cannot add edge ("
                  << id1 << "," << id2 << ")" << std::endl;
        continue;
      }

      // if (0)
      // if (abs(id1 - id2) != 1)
      // m *= 1e-6;

      // TODO(goki): transform measurement covariance by considering the
      // laserOffset to measurement between the lasers
      SE2 transf;
      transf.fromVector(p);
      e->setMeasurement(laserOffset->estimate().inverse() * transf *
                        laserOffset->estimate());
      // e->inverseMeasurement() = e->measurement().inverse();

      e->setVertex(0, v1);
      e->setVertex(1, v2);
      e->setVertex(2, laserOffset);
      if (!optimizer.addEdge(e)) {
        std::cerr << "error in adding edge " << id1 << "," << id2 << std::endl;
      }
      // std::cerr << PVAR(e->inverseMeasurement().toVector().transpose()) <<
      // std::endl; std::cerr << PVAR(e->information()) << std::endl;

    } else if (tag == "ROBOTLASER1") {
      if (previousVertex) {
        auto rl2 = std::make_shared<RobotLaser>();
        rl2->read(currentLine);
        if (!laserOffsetInitDone) {
          laserOffsetInitDone = true;
          // std::cerr << "g2o Laseroffset is " <<
          // rl2->laserParams().laserPose.toVector().transpose() << std::endl;
          laserOffset->setEstimate(rl2->laserParams().laserPose);
        }
        previousVertex->setUserData(rl2);
        previousVertex = nullptr;
      }
    }
  }

  return true;
}

bool Gm2dlIO::writeGm2dl(const std::string& filename,
                         const SparseOptimizer& optimizer) {
  std::ofstream fout(filename.c_str());
  if (!fout.good()) {
    return false;
  }
  Factory* factory = Factory::instance();

  for (const auto& it : optimizer.vertices()) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(it.second.get());
    fout << "VERTEX2 " << v->id() << " ";
    v->write(fout);
    fout << std::endl;
    auto data = v->userData();
    if (data) {  // writing the data via the factory
      const std::string tag = factory->tag(data.get());
      if (!tag.empty()) {
        fout << tag << " ";
        data->write(fout);
        fout << std::endl;
      }
    }
  }

  OptimizableGraph::EdgeContainer edgesToSave;  // sorting edges to have them in
                                                // the order of insertion again
  for (const auto& it : optimizer.edges()) {
    auto e = std::static_pointer_cast<OptimizableGraph::Edge>(it);
    edgesToSave.push_back(e);
  }
  sort(edgesToSave.begin(), edgesToSave.end(),
       OptimizableGraph::EdgeIDCompare());

  for (const auto& e : edgesToSave) {
    auto* calibEdge = dynamic_cast<EdgeSE2SensorCalib*>(e.get());
    if (calibEdge) {
      // write back in the gm2dl format
      fout << "EDGE2 " << calibEdge->vertex(0)->id() << " "
           << calibEdge->vertex(1)->id();
      Eigen::Vector3d meas = calibEdge->measurement().toVector();
      fout << " " << meas.x() << " " << meas.y() << " " << meas.z();
      const Eigen::Matrix3d& m = calibEdge->information();
      fout << " " << m(0, 0) << " " << m(0, 1) << " " << m(1, 1) << " "
           << m(2, 2) << " " << m(0, 2) << " " << m(1, 2);
      fout << std::endl;
    } else {
      // std::cerr << "Strange Edge Type: " << factory->tag(e) << std::endl;
    }
  }

  return fout.good();
}

bool Gm2dlIO::updateLaserData(SparseOptimizer& optimizer) {
  auto laserOffset =
      std::dynamic_pointer_cast<VertexSE2>(optimizer.vertex(kIdLaserpose));
  if (!laserOffset) {
    std::cerr << "Laser offset not found" << std::endl;
    return false;
  }

  for (const auto& it : optimizer.vertices()) {
    auto* v = dynamic_cast<VertexSE2*>(it.second.get());
    if (!v) continue;
    if (v->id() == kIdLaserpose) continue;
    RobotLaser* robotLaser = dynamic_cast<RobotLaser*>(v->userData().get());
    if (robotLaser) {  // writing the data via the factory
      robotLaser->setOdomPose(v->estimate());
      LaserParameters params = robotLaser->laserParams();
      params.laserPose = laserOffset->estimate();
      robotLaser->setLaserParams(params);
    }
  }
  return true;
}

int Gm2dlIO::readRobotLaser(const std::string& filename, DataQueue& queue) {
  std::ifstream is(filename.c_str());
  if (!is.good()) return 0;

  int cnt = 0;

  // parse the GM2DL file and extract the vertices, edges, and the laser data
  std::stringstream currentLine;
  while (true) {
    const int bytesRead = readLine(is, currentLine);
    if (bytesRead == -1) break;
    std::string tag;
    currentLine >> tag;
    if (tag == "ROBOTLASER1") {
      auto rl2 = std::make_shared<RobotLaser>();
      rl2->read(currentLine);
      queue.add(rl2);
      cnt++;
    }
  }
  return cnt;
}

}  // namespace g2o
