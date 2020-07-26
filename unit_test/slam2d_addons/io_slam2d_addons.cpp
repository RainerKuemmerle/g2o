// g2o - General Graph Optimization
// Copyright (C) 2014 R. Kuemmerle, G. Grisetti, W. Burgard
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

#include <sstream>

#include "g2o/types/slam2d_addons/edge_line2d.cpp"
#include "g2o/types/slam2d_addons/edge_se2_line2d.h"
#include "g2o/types/slam2d_addons/edge_se2_segment2d.h"
#include "g2o/types/slam2d_addons/edge_se2_segment2d_line.h"
#include "g2o/types/slam2d_addons/vertex_segment2d.h"
#include "g2o/types/slam2d_addons/edge_se2_segment2d_pointLine.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/io.h"

using namespace std;
using namespace g2o;

struct RandomLine2D {
  static Line2D create() { return Line2D(Vector2::Random()); }
  static bool isApprox(const Line2D& a, const Line2D& b) { return a.isApprox(b, 1e-5); }
};

TEST(IoSlam2dAddOns, ReadWriteVertexSegment2D) { readWriteVectorBasedVertex<VertexSegment2D>(); }

TEST(IoSlam2dAddOns, ReadWriteEdgeLine2D) { readWriteVectorBasedEdge<EdgeLine2D, RandomLine2D>(); }

TEST(IoSlam2dAddOns, ReadWriteEdgeSE2Line2D) { readWriteVectorBasedEdge<EdgeSE2Line2D, RandomLine2D>(); }

TEST(IoSlam2dAddOns, ReadWriteEdgeSE2Segment2D) { readWriteVectorBasedEdge<EdgeSE2Segment2D>(); }

TEST(IoSlam2dAddOns, ReadWriteEdgeSE2Segment2DLine) { readWriteVectorBasedEdge<EdgeSE2Segment2DLine>(); }

TEST(IoSlam2dAddOns, ReadWriteEdgeSE2Segment2DPointLine) { readWriteVectorBasedEdge<EdgeSE2Segment2DPointLine>(); }
