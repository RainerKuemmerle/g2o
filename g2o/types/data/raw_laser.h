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

#include "robot_data.h"
#include "laser_parameters.h"
#include "g2o_types_data_api.h"

#include <vector>

#include<Eigen/Core>
#include<Eigen/StdVector>

namespace g2o {

  /**
   * \brief Raw laser measuerement
   *
   * A raw laser measuerement. The read/write function correspond to the format of CARMEN.
   */
  class G2O_TYPES_DATA_API RawLaser : public RobotData {
    public:
      typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >      Point2DVector;

    public:
      RawLaser();
      ~RawLaser();

      virtual bool write(std::ostream& os) const;
      virtual bool read(std::istream& is);

      /**
       * computes a cartesian view of the beams (x,y).
       * @return a vector with the points of the scan in cartesian coordinates.
       */
      Point2DVector cartesian() const;

      //! the range measurements by the laser
      const std::vector<double>& ranges() const { return _ranges;}
      void setRanges(const std::vector<double>& ranges);

      //! the remission measurements by the laser
      const std::vector<double>& remissions() const { return _remissions;}
      void setRemissions(const std::vector<double>& remissions);

      //! the parameters of the laser
      const LaserParameters& laserParams() const { return _laserParams;}
      void setLaserParams(const LaserParameters& laserParams);

    protected:
      std::vector<double> _ranges;
      std::vector<double> _remissions;
      LaserParameters _laserParams;
  };

} // end namespace

#endif
