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

#include <gtest/gtest.h>

#include <tuple>

#include "g2o/types/slam3d/edge_pointxyz.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_offset.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/edge_se3_pointxyz_depth.h"
#include "g2o/types/slam3d/edge_se3_pointxyz_disparity.h"
#include "g2o/types/slam3d/edge_se3_prior.h"
#include "g2o/types/slam3d/edge_se3_xyzprior.h"
#include "g2o/types/slam3d/edge_xyz_prior.h"
#include "g2o/types/slam3d/parameter_camera.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"
#include "unit_test/test_helper/typed_basic_tests.h"

using Slam3DIoTypes = ::testing::Types<
    // without parameters
    std::tuple<g2o::EdgeSE3>, std::tuple<g2o::EdgePointXYZ>,
    std::tuple<g2o::EdgeXYZPrior>,
    // with parameters
    std::tuple<g2o::EdgeSE3Offset, g2o::ParameterSE3Offset,
               g2o::ParameterSE3Offset>,
    std::tuple<g2o::EdgeSE3PointXYZDepth, g2o::ParameterCamera>,
    std::tuple<g2o::EdgeSE3PointXYZDisparity, g2o::ParameterCamera>,
    std::tuple<g2o::EdgeSE3PointXYZ, g2o::ParameterSE3Offset>,
    std::tuple<g2o::EdgeSE3Prior, g2o::ParameterSE3Offset>,
    std::tuple<g2o::EdgeSE3XYZPrior, g2o::ParameterSE3Offset> >;
INSTANTIATE_TYPED_TEST_SUITE_P(Slam3D, FixedSizeEdgeBasicTests, Slam3DIoTypes,
                               g2o::internal::DefaultTypeNames);
