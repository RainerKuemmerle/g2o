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

#include "robot_laser.h"

#include "g2o/stuff/macros.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif

#include <iomanip>

namespace g2o {

bool RobotLaser::read(std::istream& is) {
  int type;
  number_t angle;
  number_t fov;
  number_t res;
  number_t maxrange;
  number_t acc;
  int remission_mode;
  is >> type >> angle >> fov >> res >> maxrange >> acc >> remission_mode;

  int beams;
  is >> beams;
  laserParams_ =
      LaserParameters(type, beams, angle, res, maxrange, acc, remission_mode);
  ranges_.resize(beams);
  for (int i = 0; i < beams; i++) is >> ranges_[i];

  is >> beams;
  remissions_.resize(beams);
  for (int i = 0; i < beams; i++) is >> remissions_[i];

  // special robot laser stuff
  number_t x;
  number_t y;
  number_t theta;
  is >> x >> y >> theta;
  const SE2 lp(x, y, theta);
  // cerr << "x: " << x << " y:" << y << " th:" << theta << " ";
  is >> x >> y >> theta;
  // cerr << "x: " << x << " y:" << y << " th:" << theta;
  odomPose_ = SE2(x, y, theta);
  laserParams_.laserPose = odomPose_.inverse() * lp;
  is >> laserTv_ >> laserRv_ >> forwardSafetyDist_ >> sideSafetyDist_ >>
      turnAxis_;

  // timestamp + host
  is >> timestamp_;
  is >> hostname_;
  is >> loggerTimestamp_;
  return true;
}

bool RobotLaser::write(std::ostream& os) const {
  os << laserParams_.type << " " << laserParams_.firstBeamAngle << " "
     << laserParams_.fov << " " << laserParams_.angularStep << " "
     << laserParams_.maxRange << " " << laserParams_.accuracy << " "
     << laserParams_.remissionMode << " ";
  os << ranges().size();
  for (const double i : ranges()) os << " " << i;
  os << " " << remissions_.size();
  for (const double remission : remissions_) os << " " << remission;

  // odometry pose
  Vector3 p = (odomPose_ * laserParams_.laserPose).toVector();
  os << " " << p.x() << " " << p.y() << " " << p.z();
  p = odomPose_.toVector();
  os << " " << p.x() << " " << p.y() << " " << p.z();

  // crap values
  os << FIXED(" " << laserTv_ << " " << laserRv_ << " " << forwardSafetyDist_
                  << " " << sideSafetyDist_ << " " << turnAxis_);
  os << FIXED(" " << timestamp() << " " << hostname() << " "
                  << loggerTimestamp());

  return os.good();
}

void RobotLaser::setOdomPose(const SE2& odomPose) { odomPose_ = odomPose; }

#ifdef G2O_HAVE_OPENGL
RobotLaserDrawAction::RobotLaserDrawAction()
    : DrawAction(typeid(RobotLaser).name()),
      beamsDownsampling_(nullptr),
      pointSize_(nullptr),
      maxRange_(nullptr) {}

bool RobotLaserDrawAction::refreshPropertyPtrs(
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (previousParams_) {
    beamsDownsampling_ = previousParams_->makeProperty<IntProperty>(
        typeName_ + "::BEAMS_DOWNSAMPLING", 1);
    pointSize_ = previousParams_->makeProperty<FloatProperty>(
        typeName_ + "::POINT_SIZE", 1.0F);
    maxRange_ = previousParams_->makeProperty<FloatProperty>(
        typeName_ + "::MAX_RANGE", -1.);
  } else {
    beamsDownsampling_ = nullptr;
    pointSize_ = nullptr;
    maxRange_ = nullptr;
  }
  return true;
}

bool RobotLaserDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;

  refreshPropertyPtrs(params_);
  if (!previousParams_) {
    return true;
  }
  if (show_ && !show_->value()) return true;
  auto* that = static_cast<RobotLaser*>(&element);

  RawLaser::Point2DVector points = that->cartesian();
  if (maxRange_ && maxRange_->value() >= 0) {
    // prune the cartesian points;
    RawLaser::Point2DVector npoints(points.size());
    int k = 0;
    auto r2 = std::pow(maxRange_->value(), 2);
    for (auto& point : points) {
      if (point.squaredNorm() < r2) npoints[k++] = point;
    }
    points = npoints;
    npoints.resize(k);
  }

  glPushMatrix();
  const SE2& laserPose = that->laserParams().laserPose;
  glTranslatef(static_cast<float>(laserPose.translation().x()),
               static_cast<float>(laserPose.translation().y()), 0.F);
  glRotatef(static_cast<float> RAD2DEG(laserPose.rotation().angle()), 0.F, 0.F,
            1.F);
  glColor4f(1.F, 0.F, 0.F, 0.5F);
  int step = 1;
  if (beamsDownsampling_) step = beamsDownsampling_->value();
  if (pointSize_) {
    glPointSize(pointSize_->value());
  }

  glBegin(GL_POINTS);
  for (size_t i = 0; i < points.size(); i += step) {
    glVertex3f(static_cast<float>(points[i].x()),
               static_cast<float>(points[i].y()), 0.F);
  }
  glEnd();
  glPopMatrix();

  return true;
}
#endif

}  // namespace g2o
