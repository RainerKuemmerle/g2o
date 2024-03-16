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

#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_offset.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/edge_se2_pointxy_bearing.h"
#include "g2o/types/slam2d/edge_se2_pointxy_calib.h"
#include "g2o/types/slam2d/edge_se2_pointxy_offset.h"
#include "g2o/types/slam2d/edge_se2_prior.h"
#include "g2o/types/slam2d/edge_se2_twopointsxy.h"
#include "g2o/types/slam2d/edge_xy_prior.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"
#include "unit_test/test_helper/typed_basic_tests.h"

using Slam2DIoTypes = ::testing::Types<
    // without parameters
    std::tuple<g2o::EdgeSE2>, std::tuple<g2o::EdgeSE2PointXY>,
    std::tuple<g2o::EdgeSE2PointXYBearing>, std::tuple<g2o::EdgeSE2Prior>,
    std::tuple<g2o::EdgeXYPrior>, std::tuple<g2o::EdgeSE2TwoPointsXY>,
    std::tuple<g2o::EdgeSE2PointXYCalib>,
    // with parameters
    std::tuple<g2o::EdgeSE2Offset, g2o::ParameterSE2Offset,
               g2o::ParameterSE2Offset>,
    std::tuple<g2o::EdgeSE2PointXYOffset, g2o::ParameterSE2Offset> >;
INSTANTIATE_TYPED_TEST_SUITE_P(Slam2D, FixedSizeEdgeBasicTests, Slam2DIoTypes,
                               g2o::internal::DefaultTypeNames);
