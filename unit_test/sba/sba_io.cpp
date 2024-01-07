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

#include "g2o/core/eigen_types.h"
#include "g2o/core/factory.h"
#include "g2o/types/sba/edge_project_p2mc.h"
#include "g2o/types/sba/edge_project_p2sc.h"
#include "g2o/types/sba/edge_project_stereo_xyz.h"
#include "g2o/types/sba/edge_project_stereo_xyz_onlypose.h"
#include "g2o/types/sba/edge_project_xyz.h"
#include "g2o/types/sba/edge_project_xyz_onlypose.h"
#include "g2o/types/sba/edge_sba_cam.h"
#include "g2o/types/sba/edge_sba_scale.h"
#include "g2o/types/sba/edge_se3_expmap.h"
#include "g2o/types/sba/sbacam.h"
#include "unit_test/test_helper/typed_io.h"

G2O_USE_TYPE_GROUP(slam3d)

template <>
struct g2o::internal::RandomValue<g2o::SBACam> {
  using Type = g2o::SBACam;
  static Type create() {
    g2o::SBACam result(g2o::Quaternion::UnitRandom(), g2o::Vector3::Random());
    return result;
  }
};

using SBAIoTypes =
    ::testing::Types<g2o::EdgeSE3Expmap, g2o::EdgeSBAScale, g2o::EdgeSBACam,
                     g2o::EdgeSE3ProjectXYZ, g2o::EdgeSE3ProjectXYZOnlyPose,
                     g2o::EdgeStereoSE3ProjectXYZ,
                     g2o::EdgeStereoSE3ProjectXYZOnlyPose, g2o::EdgeProjectP2SC,
                     g2o::EdgeProjectP2MC>;
INSTANTIATE_TYPED_TEST_SUITE_P(SBA, FixedSizeEdgeIO, SBAIoTypes,
                               g2o::internal::DefaultTypeNames);
