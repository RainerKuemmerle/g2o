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

#ifndef G2O_RAW_LASER_H
#define G2O_RAW_LASER_H

#include <Eigen/Core>
#include <iosfwd>
#include <vector>

#include "g2o/core/eigen_types.h"
#include "g2o_types_data_api.h"
#include "laser_parameters.h"
#include "robot_data.h"

namespace g2o {

/**
 * \brief Raw laser measurement
 *
 * A raw laser measurement. The read/write function correspond to the format of
 * CARMEN.
 */
class G2O_TYPES_DATA_API RawLaser : public RobotData {
 public:
  using Point2DVector = std::vector<Vector2>;

  RawLaser();
  ~RawLaser() override = default;

  bool write(std::ostream& os) const override;
  bool read(std::istream& is) override;

  /**
   * computes a cartesian view of the beams (x,y).
   * @return a vector with the points of the scan in cartesian coordinates.
   */
  [[nodiscard]] Point2DVector cartesian() const;

  //! the range measurements by the laser
  [[nodiscard]] const std::vector<double>& ranges() const { return ranges_; }
  void setRanges(const std::vector<double>& ranges);

  //! the remission measurements by the laser
  [[nodiscard]] const std::vector<double>& remissions() const {
    return remissions_;
  }
  void setRemissions(const std::vector<double>& remissions);

  //! the parameters of the laser
  [[nodiscard]] const LaserParameters& laserParams() const {
    return laserParams_;
  }
  void setLaserParams(const LaserParameters& laserParams);

 protected:
  std::vector<double> ranges_;
  std::vector<double> remissions_;
  LaserParameters laserParams_;
};

}  // namespace g2o

#endif
