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

#include "g2o/core/base_fixed_sized_edge.h"

#include "g2o/core/base_variable_sized_edge.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "gtest/gtest.h"

class Edge3Constant : public g2o::BaseFixedSizedEdge<2, g2o::Vector2, g2o::VertexSE2,
                                                     g2o::VertexSE2, g2o::VertexPointXY> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Edge3Constant()
      : g2o::BaseFixedSizedEdge<2, g2o::Vector2, g2o::VertexSE2, g2o::VertexSE2,
                                g2o::VertexPointXY>(){};
  void computeError() {
    const auto a = static_cast<const g2o::VertexSE2*>(_vertices[0])->estimate();
    const auto b = static_cast<const g2o::VertexSE2*>(_vertices[1])->estimate();
    const auto c = static_cast<const g2o::VertexPointXY*>(_vertices[2])->estimate();
    _error = (a * b * c - _measurement).eval();
  }
  virtual bool read(std::istream&) { return false; };
  virtual bool write(std::ostream&) const { return false; };
};

class Edge3Dynamic : public g2o::BaseVariableSizedEdge<2, g2o::Vector2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Edge3Dynamic() : g2o::BaseVariableSizedEdge<2, g2o::Vector2>() { resize(3); };
  void computeError() {
    const auto a = static_cast<const g2o::VertexSE2*>(_vertices[0])->estimate();
    const auto b = static_cast<const g2o::VertexSE2*>(_vertices[1])->estimate();
    const auto c = static_cast<const g2o::VertexPointXY*>(_vertices[2])->estimate();
    _error = (a * b * c - _measurement).eval();
  }
  virtual bool read(std::istream&) { return false; };
  virtual bool write(std::ostream&) const { return false; };
};

TEST(General, IndexToPairToIndex) {
//#define TEST_ALL_PAIRS
  using g2o::internal::index_to_pair;
  using g2o::internal::pair_to_index;

  // brutal way of testing
  ASSERT_EQ(index_to_pair<0>(), g2o::internal::TrivialPair(0, 1));
  ASSERT_EQ(index_to_pair<1>(), g2o::internal::TrivialPair(0, 2));
  ASSERT_EQ(index_to_pair<2>(), g2o::internal::TrivialPair(1, 2));
  ASSERT_EQ(index_to_pair<3>(), g2o::internal::TrivialPair(0, 3));
  ASSERT_EQ(index_to_pair<4>(), g2o::internal::TrivialPair(1, 3));
  ASSERT_EQ(index_to_pair<5>(), g2o::internal::TrivialPair(2, 3));
  ASSERT_EQ(index_to_pair<6>(), g2o::internal::TrivialPair(0, 4));
  ASSERT_EQ(index_to_pair<7>(), g2o::internal::TrivialPair(1, 4));
  ASSERT_EQ(index_to_pair<8>(), g2o::internal::TrivialPair(2, 4));
  ASSERT_EQ(index_to_pair<9>(), g2o::internal::TrivialPair(3, 4));
  ASSERT_EQ(index_to_pair<10>(), g2o::internal::TrivialPair(0, 5));
  ASSERT_EQ(index_to_pair<11>(), g2o::internal::TrivialPair(1, 5));
  ASSERT_EQ(index_to_pair<12>(), g2o::internal::TrivialPair(2, 5));
  ASSERT_EQ(index_to_pair<13>(), g2o::internal::TrivialPair(3, 5));
  ASSERT_EQ(index_to_pair<14>(), g2o::internal::TrivialPair(4, 5));
  ASSERT_EQ(index_to_pair<15>(), g2o::internal::TrivialPair(0, 6));
  ASSERT_EQ(index_to_pair<16>(), g2o::internal::TrivialPair(1, 6));
  ASSERT_EQ(index_to_pair<17>(), g2o::internal::TrivialPair(2, 6));
  ASSERT_EQ(index_to_pair<18>(), g2o::internal::TrivialPair(3, 6));
  ASSERT_EQ(index_to_pair<19>(), g2o::internal::TrivialPair(4, 6));
  ASSERT_EQ(index_to_pair<20>(), g2o::internal::TrivialPair(5, 6));
  ASSERT_EQ(index_to_pair<21>(), g2o::internal::TrivialPair(0, 7));
  ASSERT_EQ(index_to_pair<22>(), g2o::internal::TrivialPair(1, 7));
  ASSERT_EQ(index_to_pair<23>(), g2o::internal::TrivialPair(2, 7));
  ASSERT_EQ(index_to_pair<24>(), g2o::internal::TrivialPair(3, 7));
  ASSERT_EQ(index_to_pair<25>(), g2o::internal::TrivialPair(4, 7));
  ASSERT_EQ(index_to_pair<26>(), g2o::internal::TrivialPair(5, 7));
  ASSERT_EQ(index_to_pair<27>(), g2o::internal::TrivialPair(6, 7));
  ASSERT_EQ(index_to_pair<28>(), g2o::internal::TrivialPair(0, 8));
  ASSERT_EQ(index_to_pair<29>(), g2o::internal::TrivialPair(1, 8));
  ASSERT_EQ(index_to_pair<30>(), g2o::internal::TrivialPair(2, 8));
  ASSERT_EQ(index_to_pair<31>(), g2o::internal::TrivialPair(3, 8));
  ASSERT_EQ(index_to_pair<32>(), g2o::internal::TrivialPair(4, 8));
  ASSERT_EQ(index_to_pair<33>(), g2o::internal::TrivialPair(5, 8));
  ASSERT_EQ(index_to_pair<34>(), g2o::internal::TrivialPair(6, 8));
  ASSERT_EQ(index_to_pair<35>(), g2o::internal::TrivialPair(7, 8));
  ASSERT_EQ(index_to_pair<36>(), g2o::internal::TrivialPair(0, 9));
  ASSERT_EQ(index_to_pair<37>(), g2o::internal::TrivialPair(1, 9));
  ASSERT_EQ(index_to_pair<38>(), g2o::internal::TrivialPair(2, 9));
  ASSERT_EQ(index_to_pair<39>(), g2o::internal::TrivialPair(3, 9));
  ASSERT_EQ(index_to_pair<40>(), g2o::internal::TrivialPair(4, 9));
  ASSERT_EQ(index_to_pair<41>(), g2o::internal::TrivialPair(5, 9));
  ASSERT_EQ(index_to_pair<42>(), g2o::internal::TrivialPair(6, 9));
  ASSERT_EQ(index_to_pair<43>(), g2o::internal::TrivialPair(7, 9));
  ASSERT_EQ(index_to_pair<44>(), g2o::internal::TrivialPair(8, 9));
  ASSERT_EQ(index_to_pair<45>(), g2o::internal::TrivialPair(0, 10));
  ASSERT_EQ(index_to_pair<46>(), g2o::internal::TrivialPair(1, 10));
  ASSERT_EQ(index_to_pair<47>(), g2o::internal::TrivialPair(2, 10));
  ASSERT_EQ(index_to_pair<48>(), g2o::internal::TrivialPair(3, 10));
  ASSERT_EQ(index_to_pair<49>(), g2o::internal::TrivialPair(4, 10));
  ASSERT_EQ(index_to_pair<50>(), g2o::internal::TrivialPair(5, 10));
  ASSERT_EQ(index_to_pair<51>(), g2o::internal::TrivialPair(6, 10));
  ASSERT_EQ(index_to_pair<52>(), g2o::internal::TrivialPair(7, 10));
  ASSERT_EQ(index_to_pair<53>(), g2o::internal::TrivialPair(8, 10));
  ASSERT_EQ(index_to_pair<54>(), g2o::internal::TrivialPair(9, 10));
#ifdef TEST_ALL_PAIRS
  ASSERT_EQ(index_to_pair<55>(), g2o::internal::TrivialPair(0, 11));
  ASSERT_EQ(index_to_pair<56>(), g2o::internal::TrivialPair(1, 11));
  ASSERT_EQ(index_to_pair<57>(), g2o::internal::TrivialPair(2, 11));
  ASSERT_EQ(index_to_pair<58>(), g2o::internal::TrivialPair(3, 11));
  ASSERT_EQ(index_to_pair<59>(), g2o::internal::TrivialPair(4, 11));
  ASSERT_EQ(index_to_pair<60>(), g2o::internal::TrivialPair(5, 11));
  ASSERT_EQ(index_to_pair<61>(), g2o::internal::TrivialPair(6, 11));
  ASSERT_EQ(index_to_pair<62>(), g2o::internal::TrivialPair(7, 11));
  ASSERT_EQ(index_to_pair<63>(), g2o::internal::TrivialPair(8, 11));
  ASSERT_EQ(index_to_pair<64>(), g2o::internal::TrivialPair(9, 11));
  ASSERT_EQ(index_to_pair<65>(), g2o::internal::TrivialPair(10, 11));
  ASSERT_EQ(index_to_pair<66>(), g2o::internal::TrivialPair(0, 12));
  ASSERT_EQ(index_to_pair<67>(), g2o::internal::TrivialPair(1, 12));
  ASSERT_EQ(index_to_pair<68>(), g2o::internal::TrivialPair(2, 12));
  ASSERT_EQ(index_to_pair<69>(), g2o::internal::TrivialPair(3, 12));
  ASSERT_EQ(index_to_pair<70>(), g2o::internal::TrivialPair(4, 12));
  ASSERT_EQ(index_to_pair<71>(), g2o::internal::TrivialPair(5, 12));
  ASSERT_EQ(index_to_pair<72>(), g2o::internal::TrivialPair(6, 12));
  ASSERT_EQ(index_to_pair<73>(), g2o::internal::TrivialPair(7, 12));
  ASSERT_EQ(index_to_pair<74>(), g2o::internal::TrivialPair(8, 12));
  ASSERT_EQ(index_to_pair<75>(), g2o::internal::TrivialPair(9, 12));
  ASSERT_EQ(index_to_pair<76>(), g2o::internal::TrivialPair(10, 12));
  ASSERT_EQ(index_to_pair<77>(), g2o::internal::TrivialPair(11, 12));
  ASSERT_EQ(index_to_pair<78>(), g2o::internal::TrivialPair(0, 13));
  ASSERT_EQ(index_to_pair<79>(), g2o::internal::TrivialPair(1, 13));
  ASSERT_EQ(index_to_pair<80>(), g2o::internal::TrivialPair(2, 13));
  ASSERT_EQ(index_to_pair<81>(), g2o::internal::TrivialPair(3, 13));
  ASSERT_EQ(index_to_pair<82>(), g2o::internal::TrivialPair(4, 13));
  ASSERT_EQ(index_to_pair<83>(), g2o::internal::TrivialPair(5, 13));
  ASSERT_EQ(index_to_pair<84>(), g2o::internal::TrivialPair(6, 13));
  ASSERT_EQ(index_to_pair<85>(), g2o::internal::TrivialPair(7, 13));
  ASSERT_EQ(index_to_pair<86>(), g2o::internal::TrivialPair(8, 13));
  ASSERT_EQ(index_to_pair<87>(), g2o::internal::TrivialPair(9, 13));
  ASSERT_EQ(index_to_pair<88>(), g2o::internal::TrivialPair(10, 13));
  ASSERT_EQ(index_to_pair<89>(), g2o::internal::TrivialPair(11, 13));
  ASSERT_EQ(index_to_pair<90>(), g2o::internal::TrivialPair(12, 13));
  ASSERT_EQ(index_to_pair<91>(), g2o::internal::TrivialPair(0, 14));
  ASSERT_EQ(index_to_pair<92>(), g2o::internal::TrivialPair(1, 14));
  ASSERT_EQ(index_to_pair<93>(), g2o::internal::TrivialPair(2, 14));
  ASSERT_EQ(index_to_pair<94>(), g2o::internal::TrivialPair(3, 14));
  ASSERT_EQ(index_to_pair<95>(), g2o::internal::TrivialPair(4, 14));
  ASSERT_EQ(index_to_pair<96>(), g2o::internal::TrivialPair(5, 14));
  ASSERT_EQ(index_to_pair<97>(), g2o::internal::TrivialPair(6, 14));
  ASSERT_EQ(index_to_pair<98>(), g2o::internal::TrivialPair(7, 14));
  ASSERT_EQ(index_to_pair<99>(), g2o::internal::TrivialPair(8, 14));
  ASSERT_EQ(index_to_pair<100>(), g2o::internal::TrivialPair(9, 14));
  ASSERT_EQ(index_to_pair<101>(), g2o::internal::TrivialPair(10, 14));
  ASSERT_EQ(index_to_pair<102>(), g2o::internal::TrivialPair(11, 14));
  ASSERT_EQ(index_to_pair<103>(), g2o::internal::TrivialPair(12, 14));
  ASSERT_EQ(index_to_pair<104>(), g2o::internal::TrivialPair(13, 14));
  ASSERT_EQ(index_to_pair<105>(), g2o::internal::TrivialPair(0, 15));
  ASSERT_EQ(index_to_pair<106>(), g2o::internal::TrivialPair(1, 15));
  ASSERT_EQ(index_to_pair<107>(), g2o::internal::TrivialPair(2, 15));
  ASSERT_EQ(index_to_pair<108>(), g2o::internal::TrivialPair(3, 15));
  ASSERT_EQ(index_to_pair<109>(), g2o::internal::TrivialPair(4, 15));
  ASSERT_EQ(index_to_pair<110>(), g2o::internal::TrivialPair(5, 15));
  ASSERT_EQ(index_to_pair<111>(), g2o::internal::TrivialPair(6, 15));
  ASSERT_EQ(index_to_pair<112>(), g2o::internal::TrivialPair(7, 15));
  ASSERT_EQ(index_to_pair<113>(), g2o::internal::TrivialPair(8, 15));
  ASSERT_EQ(index_to_pair<114>(), g2o::internal::TrivialPair(9, 15));
  ASSERT_EQ(index_to_pair<115>(), g2o::internal::TrivialPair(10, 15));
  ASSERT_EQ(index_to_pair<116>(), g2o::internal::TrivialPair(11, 15));
  ASSERT_EQ(index_to_pair<117>(), g2o::internal::TrivialPair(12, 15));
  ASSERT_EQ(index_to_pair<118>(), g2o::internal::TrivialPair(13, 15));
  ASSERT_EQ(index_to_pair<119>(), g2o::internal::TrivialPair(14, 15));
  ASSERT_EQ(index_to_pair<120>(), g2o::internal::TrivialPair(0, 16));
  ASSERT_EQ(index_to_pair<121>(), g2o::internal::TrivialPair(1, 16));
  ASSERT_EQ(index_to_pair<122>(), g2o::internal::TrivialPair(2, 16));
  ASSERT_EQ(index_to_pair<123>(), g2o::internal::TrivialPair(3, 16));
  ASSERT_EQ(index_to_pair<124>(), g2o::internal::TrivialPair(4, 16));
  ASSERT_EQ(index_to_pair<125>(), g2o::internal::TrivialPair(5, 16));
  ASSERT_EQ(index_to_pair<126>(), g2o::internal::TrivialPair(6, 16));
  ASSERT_EQ(index_to_pair<127>(), g2o::internal::TrivialPair(7, 16));
  ASSERT_EQ(index_to_pair<128>(), g2o::internal::TrivialPair(8, 16));
  ASSERT_EQ(index_to_pair<129>(), g2o::internal::TrivialPair(9, 16));
  ASSERT_EQ(index_to_pair<130>(), g2o::internal::TrivialPair(10, 16));
  ASSERT_EQ(index_to_pair<131>(), g2o::internal::TrivialPair(11, 16));
  ASSERT_EQ(index_to_pair<132>(), g2o::internal::TrivialPair(12, 16));
  ASSERT_EQ(index_to_pair<133>(), g2o::internal::TrivialPair(13, 16));
  ASSERT_EQ(index_to_pair<134>(), g2o::internal::TrivialPair(14, 16));
  ASSERT_EQ(index_to_pair<135>(), g2o::internal::TrivialPair(15, 16));
  ASSERT_EQ(index_to_pair<136>(), g2o::internal::TrivialPair(0, 17));
  ASSERT_EQ(index_to_pair<137>(), g2o::internal::TrivialPair(1, 17));
  ASSERT_EQ(index_to_pair<138>(), g2o::internal::TrivialPair(2, 17));
  ASSERT_EQ(index_to_pair<139>(), g2o::internal::TrivialPair(3, 17));
  ASSERT_EQ(index_to_pair<140>(), g2o::internal::TrivialPair(4, 17));
  ASSERT_EQ(index_to_pair<141>(), g2o::internal::TrivialPair(5, 17));
  ASSERT_EQ(index_to_pair<142>(), g2o::internal::TrivialPair(6, 17));
  ASSERT_EQ(index_to_pair<143>(), g2o::internal::TrivialPair(7, 17));
  ASSERT_EQ(index_to_pair<144>(), g2o::internal::TrivialPair(8, 17));
  ASSERT_EQ(index_to_pair<145>(), g2o::internal::TrivialPair(9, 17));
  ASSERT_EQ(index_to_pair<146>(), g2o::internal::TrivialPair(10, 17));
  ASSERT_EQ(index_to_pair<147>(), g2o::internal::TrivialPair(11, 17));
  ASSERT_EQ(index_to_pair<148>(), g2o::internal::TrivialPair(12, 17));
  ASSERT_EQ(index_to_pair<149>(), g2o::internal::TrivialPair(13, 17));
  ASSERT_EQ(index_to_pair<150>(), g2o::internal::TrivialPair(14, 17));
  ASSERT_EQ(index_to_pair<151>(), g2o::internal::TrivialPair(15, 17));
  ASSERT_EQ(index_to_pair<152>(), g2o::internal::TrivialPair(16, 17));
  ASSERT_EQ(index_to_pair<153>(), g2o::internal::TrivialPair(0, 18));
  ASSERT_EQ(index_to_pair<154>(), g2o::internal::TrivialPair(1, 18));
  ASSERT_EQ(index_to_pair<155>(), g2o::internal::TrivialPair(2, 18));
  ASSERT_EQ(index_to_pair<156>(), g2o::internal::TrivialPair(3, 18));
  ASSERT_EQ(index_to_pair<157>(), g2o::internal::TrivialPair(4, 18));
  ASSERT_EQ(index_to_pair<158>(), g2o::internal::TrivialPair(5, 18));
  ASSERT_EQ(index_to_pair<159>(), g2o::internal::TrivialPair(6, 18));
  ASSERT_EQ(index_to_pair<160>(), g2o::internal::TrivialPair(7, 18));
  ASSERT_EQ(index_to_pair<161>(), g2o::internal::TrivialPair(8, 18));
  ASSERT_EQ(index_to_pair<162>(), g2o::internal::TrivialPair(9, 18));
  ASSERT_EQ(index_to_pair<163>(), g2o::internal::TrivialPair(10, 18));
  ASSERT_EQ(index_to_pair<164>(), g2o::internal::TrivialPair(11, 18));
  ASSERT_EQ(index_to_pair<165>(), g2o::internal::TrivialPair(12, 18));
  ASSERT_EQ(index_to_pair<166>(), g2o::internal::TrivialPair(13, 18));
  ASSERT_EQ(index_to_pair<167>(), g2o::internal::TrivialPair(14, 18));
  ASSERT_EQ(index_to_pair<168>(), g2o::internal::TrivialPair(15, 18));
  ASSERT_EQ(index_to_pair<169>(), g2o::internal::TrivialPair(16, 18));
  ASSERT_EQ(index_to_pair<170>(), g2o::internal::TrivialPair(17, 18));
  ASSERT_EQ(index_to_pair<171>(), g2o::internal::TrivialPair(0, 19));
  ASSERT_EQ(index_to_pair<172>(), g2o::internal::TrivialPair(1, 19));
  ASSERT_EQ(index_to_pair<173>(), g2o::internal::TrivialPair(2, 19));
  ASSERT_EQ(index_to_pair<174>(), g2o::internal::TrivialPair(3, 19));
  ASSERT_EQ(index_to_pair<175>(), g2o::internal::TrivialPair(4, 19));
  ASSERT_EQ(index_to_pair<176>(), g2o::internal::TrivialPair(5, 19));
  ASSERT_EQ(index_to_pair<177>(), g2o::internal::TrivialPair(6, 19));
  ASSERT_EQ(index_to_pair<178>(), g2o::internal::TrivialPair(7, 19));
  ASSERT_EQ(index_to_pair<179>(), g2o::internal::TrivialPair(8, 19));
  ASSERT_EQ(index_to_pair<180>(), g2o::internal::TrivialPair(9, 19));
  ASSERT_EQ(index_to_pair<181>(), g2o::internal::TrivialPair(10, 19));
  ASSERT_EQ(index_to_pair<182>(), g2o::internal::TrivialPair(11, 19));
  ASSERT_EQ(index_to_pair<183>(), g2o::internal::TrivialPair(12, 19));
  ASSERT_EQ(index_to_pair<184>(), g2o::internal::TrivialPair(13, 19));
  ASSERT_EQ(index_to_pair<185>(), g2o::internal::TrivialPair(14, 19));
  ASSERT_EQ(index_to_pair<186>(), g2o::internal::TrivialPair(15, 19));
  ASSERT_EQ(index_to_pair<187>(), g2o::internal::TrivialPair(16, 19));
  ASSERT_EQ(index_to_pair<188>(), g2o::internal::TrivialPair(17, 19));
  ASSERT_EQ(index_to_pair<189>(), g2o::internal::TrivialPair(18, 19));
  ASSERT_EQ(index_to_pair<190>(), g2o::internal::TrivialPair(0, 20));
  ASSERT_EQ(index_to_pair<191>(), g2o::internal::TrivialPair(1, 20));
  ASSERT_EQ(index_to_pair<192>(), g2o::internal::TrivialPair(2, 20));
  ASSERT_EQ(index_to_pair<193>(), g2o::internal::TrivialPair(3, 20));
  ASSERT_EQ(index_to_pair<194>(), g2o::internal::TrivialPair(4, 20));
  ASSERT_EQ(index_to_pair<195>(), g2o::internal::TrivialPair(5, 20));
  ASSERT_EQ(index_to_pair<196>(), g2o::internal::TrivialPair(6, 20));
  ASSERT_EQ(index_to_pair<197>(), g2o::internal::TrivialPair(7, 20));
  ASSERT_EQ(index_to_pair<198>(), g2o::internal::TrivialPair(8, 20));
  ASSERT_EQ(index_to_pair<199>(), g2o::internal::TrivialPair(9, 20));
  ASSERT_EQ(index_to_pair<200>(), g2o::internal::TrivialPair(10, 20));
  ASSERT_EQ(index_to_pair<201>(), g2o::internal::TrivialPair(11, 20));
  ASSERT_EQ(index_to_pair<202>(), g2o::internal::TrivialPair(12, 20));
  ASSERT_EQ(index_to_pair<203>(), g2o::internal::TrivialPair(13, 20));
  ASSERT_EQ(index_to_pair<204>(), g2o::internal::TrivialPair(14, 20));
  ASSERT_EQ(index_to_pair<205>(), g2o::internal::TrivialPair(15, 20));
  ASSERT_EQ(index_to_pair<206>(), g2o::internal::TrivialPair(16, 20));
  ASSERT_EQ(index_to_pair<207>(), g2o::internal::TrivialPair(17, 20));
  ASSERT_EQ(index_to_pair<208>(), g2o::internal::TrivialPair(18, 20));
  ASSERT_EQ(index_to_pair<209>(), g2o::internal::TrivialPair(19, 20));
  ASSERT_EQ(index_to_pair<210>(), g2o::internal::TrivialPair(0, 21));
  ASSERT_EQ(index_to_pair<211>(), g2o::internal::TrivialPair(1, 21));
  ASSERT_EQ(index_to_pair<212>(), g2o::internal::TrivialPair(2, 21));
  ASSERT_EQ(index_to_pair<213>(), g2o::internal::TrivialPair(3, 21));
  ASSERT_EQ(index_to_pair<214>(), g2o::internal::TrivialPair(4, 21));
  ASSERT_EQ(index_to_pair<215>(), g2o::internal::TrivialPair(5, 21));
  ASSERT_EQ(index_to_pair<216>(), g2o::internal::TrivialPair(6, 21));
  ASSERT_EQ(index_to_pair<217>(), g2o::internal::TrivialPair(7, 21));
  ASSERT_EQ(index_to_pair<218>(), g2o::internal::TrivialPair(8, 21));
  ASSERT_EQ(index_to_pair<219>(), g2o::internal::TrivialPair(9, 21));
  ASSERT_EQ(index_to_pair<220>(), g2o::internal::TrivialPair(10, 21));
  ASSERT_EQ(index_to_pair<221>(), g2o::internal::TrivialPair(11, 21));
  ASSERT_EQ(index_to_pair<222>(), g2o::internal::TrivialPair(12, 21));
  ASSERT_EQ(index_to_pair<223>(), g2o::internal::TrivialPair(13, 21));
  ASSERT_EQ(index_to_pair<224>(), g2o::internal::TrivialPair(14, 21));
  ASSERT_EQ(index_to_pair<225>(), g2o::internal::TrivialPair(15, 21));
  ASSERT_EQ(index_to_pair<226>(), g2o::internal::TrivialPair(16, 21));
  ASSERT_EQ(index_to_pair<227>(), g2o::internal::TrivialPair(17, 21));
  ASSERT_EQ(index_to_pair<228>(), g2o::internal::TrivialPair(18, 21));
  ASSERT_EQ(index_to_pair<229>(), g2o::internal::TrivialPair(19, 21));
  ASSERT_EQ(index_to_pair<230>(), g2o::internal::TrivialPair(20, 21));
  ASSERT_EQ(index_to_pair<231>(), g2o::internal::TrivialPair(0, 22));
  ASSERT_EQ(index_to_pair<232>(), g2o::internal::TrivialPair(1, 22));
  ASSERT_EQ(index_to_pair<233>(), g2o::internal::TrivialPair(2, 22));
  ASSERT_EQ(index_to_pair<234>(), g2o::internal::TrivialPair(3, 22));
  ASSERT_EQ(index_to_pair<235>(), g2o::internal::TrivialPair(4, 22));
  ASSERT_EQ(index_to_pair<236>(), g2o::internal::TrivialPair(5, 22));
  ASSERT_EQ(index_to_pair<237>(), g2o::internal::TrivialPair(6, 22));
  ASSERT_EQ(index_to_pair<238>(), g2o::internal::TrivialPair(7, 22));
  ASSERT_EQ(index_to_pair<239>(), g2o::internal::TrivialPair(8, 22));
  ASSERT_EQ(index_to_pair<240>(), g2o::internal::TrivialPair(9, 22));
  ASSERT_EQ(index_to_pair<241>(), g2o::internal::TrivialPair(10, 22));
  ASSERT_EQ(index_to_pair<242>(), g2o::internal::TrivialPair(11, 22));
  ASSERT_EQ(index_to_pair<243>(), g2o::internal::TrivialPair(12, 22));
  ASSERT_EQ(index_to_pair<244>(), g2o::internal::TrivialPair(13, 22));
  ASSERT_EQ(index_to_pair<245>(), g2o::internal::TrivialPair(14, 22));
  ASSERT_EQ(index_to_pair<246>(), g2o::internal::TrivialPair(15, 22));
  ASSERT_EQ(index_to_pair<247>(), g2o::internal::TrivialPair(16, 22));
  ASSERT_EQ(index_to_pair<248>(), g2o::internal::TrivialPair(17, 22));
  ASSERT_EQ(index_to_pair<249>(), g2o::internal::TrivialPair(18, 22));
  ASSERT_EQ(index_to_pair<250>(), g2o::internal::TrivialPair(19, 22));
  ASSERT_EQ(index_to_pair<251>(), g2o::internal::TrivialPair(20, 22));
  ASSERT_EQ(index_to_pair<252>(), g2o::internal::TrivialPair(21, 22));
  ASSERT_EQ(index_to_pair<253>(), g2o::internal::TrivialPair(0, 23));
  ASSERT_EQ(index_to_pair<254>(), g2o::internal::TrivialPair(1, 23));
  ASSERT_EQ(index_to_pair<255>(), g2o::internal::TrivialPair(2, 23));
  ASSERT_EQ(index_to_pair<256>(), g2o::internal::TrivialPair(3, 23));
  ASSERT_EQ(index_to_pair<257>(), g2o::internal::TrivialPair(4, 23));
  ASSERT_EQ(index_to_pair<258>(), g2o::internal::TrivialPair(5, 23));
  ASSERT_EQ(index_to_pair<259>(), g2o::internal::TrivialPair(6, 23));
  ASSERT_EQ(index_to_pair<260>(), g2o::internal::TrivialPair(7, 23));
  ASSERT_EQ(index_to_pair<261>(), g2o::internal::TrivialPair(8, 23));
  ASSERT_EQ(index_to_pair<262>(), g2o::internal::TrivialPair(9, 23));
  ASSERT_EQ(index_to_pair<263>(), g2o::internal::TrivialPair(10, 23));
  ASSERT_EQ(index_to_pair<264>(), g2o::internal::TrivialPair(11, 23));
  ASSERT_EQ(index_to_pair<265>(), g2o::internal::TrivialPair(12, 23));
  ASSERT_EQ(index_to_pair<266>(), g2o::internal::TrivialPair(13, 23));
  ASSERT_EQ(index_to_pair<267>(), g2o::internal::TrivialPair(14, 23));
  ASSERT_EQ(index_to_pair<268>(), g2o::internal::TrivialPair(15, 23));
  ASSERT_EQ(index_to_pair<269>(), g2o::internal::TrivialPair(16, 23));
  ASSERT_EQ(index_to_pair<270>(), g2o::internal::TrivialPair(17, 23));
  ASSERT_EQ(index_to_pair<271>(), g2o::internal::TrivialPair(18, 23));
  ASSERT_EQ(index_to_pair<272>(), g2o::internal::TrivialPair(19, 23));
  ASSERT_EQ(index_to_pair<273>(), g2o::internal::TrivialPair(20, 23));
  ASSERT_EQ(index_to_pair<274>(), g2o::internal::TrivialPair(21, 23));
  ASSERT_EQ(index_to_pair<275>(), g2o::internal::TrivialPair(22, 23));
  ASSERT_EQ(index_to_pair<276>(), g2o::internal::TrivialPair(0, 24));
  ASSERT_EQ(index_to_pair<277>(), g2o::internal::TrivialPair(1, 24));
  ASSERT_EQ(index_to_pair<278>(), g2o::internal::TrivialPair(2, 24));
  ASSERT_EQ(index_to_pair<279>(), g2o::internal::TrivialPair(3, 24));
  ASSERT_EQ(index_to_pair<280>(), g2o::internal::TrivialPair(4, 24));
  ASSERT_EQ(index_to_pair<281>(), g2o::internal::TrivialPair(5, 24));
  ASSERT_EQ(index_to_pair<282>(), g2o::internal::TrivialPair(6, 24));
  ASSERT_EQ(index_to_pair<283>(), g2o::internal::TrivialPair(7, 24));
  ASSERT_EQ(index_to_pair<284>(), g2o::internal::TrivialPair(8, 24));
  ASSERT_EQ(index_to_pair<285>(), g2o::internal::TrivialPair(9, 24));
  ASSERT_EQ(index_to_pair<286>(), g2o::internal::TrivialPair(10, 24));
  ASSERT_EQ(index_to_pair<287>(), g2o::internal::TrivialPair(11, 24));
  ASSERT_EQ(index_to_pair<288>(), g2o::internal::TrivialPair(12, 24));
  ASSERT_EQ(index_to_pair<289>(), g2o::internal::TrivialPair(13, 24));
  ASSERT_EQ(index_to_pair<290>(), g2o::internal::TrivialPair(14, 24));
  ASSERT_EQ(index_to_pair<291>(), g2o::internal::TrivialPair(15, 24));
  ASSERT_EQ(index_to_pair<292>(), g2o::internal::TrivialPair(16, 24));
  ASSERT_EQ(index_to_pair<293>(), g2o::internal::TrivialPair(17, 24));
  ASSERT_EQ(index_to_pair<294>(), g2o::internal::TrivialPair(18, 24));
  ASSERT_EQ(index_to_pair<295>(), g2o::internal::TrivialPair(19, 24));
  ASSERT_EQ(index_to_pair<296>(), g2o::internal::TrivialPair(20, 24));
  ASSERT_EQ(index_to_pair<297>(), g2o::internal::TrivialPair(21, 24));
  ASSERT_EQ(index_to_pair<298>(), g2o::internal::TrivialPair(22, 24));
  ASSERT_EQ(index_to_pair<299>(), g2o::internal::TrivialPair(23, 24));
  ASSERT_EQ(index_to_pair<300>(), g2o::internal::TrivialPair(0, 25));
  ASSERT_EQ(index_to_pair<301>(), g2o::internal::TrivialPair(1, 25));
  ASSERT_EQ(index_to_pair<302>(), g2o::internal::TrivialPair(2, 25));
  ASSERT_EQ(index_to_pair<303>(), g2o::internal::TrivialPair(3, 25));
  ASSERT_EQ(index_to_pair<304>(), g2o::internal::TrivialPair(4, 25));
  ASSERT_EQ(index_to_pair<305>(), g2o::internal::TrivialPair(5, 25));
  ASSERT_EQ(index_to_pair<306>(), g2o::internal::TrivialPair(6, 25));
  ASSERT_EQ(index_to_pair<307>(), g2o::internal::TrivialPair(7, 25));
  ASSERT_EQ(index_to_pair<308>(), g2o::internal::TrivialPair(8, 25));
  ASSERT_EQ(index_to_pair<309>(), g2o::internal::TrivialPair(9, 25));
  ASSERT_EQ(index_to_pair<310>(), g2o::internal::TrivialPair(10, 25));
  ASSERT_EQ(index_to_pair<311>(), g2o::internal::TrivialPair(11, 25));
  ASSERT_EQ(index_to_pair<312>(), g2o::internal::TrivialPair(12, 25));
  ASSERT_EQ(index_to_pair<313>(), g2o::internal::TrivialPair(13, 25));
  ASSERT_EQ(index_to_pair<314>(), g2o::internal::TrivialPair(14, 25));
  ASSERT_EQ(index_to_pair<315>(), g2o::internal::TrivialPair(15, 25));
  ASSERT_EQ(index_to_pair<316>(), g2o::internal::TrivialPair(16, 25));
  ASSERT_EQ(index_to_pair<317>(), g2o::internal::TrivialPair(17, 25));
  ASSERT_EQ(index_to_pair<318>(), g2o::internal::TrivialPair(18, 25));
  ASSERT_EQ(index_to_pair<319>(), g2o::internal::TrivialPair(19, 25));
  ASSERT_EQ(index_to_pair<320>(), g2o::internal::TrivialPair(20, 25));
  ASSERT_EQ(index_to_pair<321>(), g2o::internal::TrivialPair(21, 25));
  ASSERT_EQ(index_to_pair<322>(), g2o::internal::TrivialPair(22, 25));
  ASSERT_EQ(index_to_pair<323>(), g2o::internal::TrivialPair(23, 25));
  ASSERT_EQ(index_to_pair<324>(), g2o::internal::TrivialPair(24, 25));
  ASSERT_EQ(index_to_pair<325>(), g2o::internal::TrivialPair(0, 26));
  ASSERT_EQ(index_to_pair<326>(), g2o::internal::TrivialPair(1, 26));
  ASSERT_EQ(index_to_pair<327>(), g2o::internal::TrivialPair(2, 26));
  ASSERT_EQ(index_to_pair<328>(), g2o::internal::TrivialPair(3, 26));
  ASSERT_EQ(index_to_pair<329>(), g2o::internal::TrivialPair(4, 26));
  ASSERT_EQ(index_to_pair<330>(), g2o::internal::TrivialPair(5, 26));
  ASSERT_EQ(index_to_pair<331>(), g2o::internal::TrivialPair(6, 26));
  ASSERT_EQ(index_to_pair<332>(), g2o::internal::TrivialPair(7, 26));
  ASSERT_EQ(index_to_pair<333>(), g2o::internal::TrivialPair(8, 26));
  ASSERT_EQ(index_to_pair<334>(), g2o::internal::TrivialPair(9, 26));
  ASSERT_EQ(index_to_pair<335>(), g2o::internal::TrivialPair(10, 26));
  ASSERT_EQ(index_to_pair<336>(), g2o::internal::TrivialPair(11, 26));
  ASSERT_EQ(index_to_pair<337>(), g2o::internal::TrivialPair(12, 26));
  ASSERT_EQ(index_to_pair<338>(), g2o::internal::TrivialPair(13, 26));
  ASSERT_EQ(index_to_pair<339>(), g2o::internal::TrivialPair(14, 26));
  ASSERT_EQ(index_to_pair<340>(), g2o::internal::TrivialPair(15, 26));
  ASSERT_EQ(index_to_pair<341>(), g2o::internal::TrivialPair(16, 26));
  ASSERT_EQ(index_to_pair<342>(), g2o::internal::TrivialPair(17, 26));
  ASSERT_EQ(index_to_pair<343>(), g2o::internal::TrivialPair(18, 26));
  ASSERT_EQ(index_to_pair<344>(), g2o::internal::TrivialPair(19, 26));
  ASSERT_EQ(index_to_pair<345>(), g2o::internal::TrivialPair(20, 26));
  ASSERT_EQ(index_to_pair<346>(), g2o::internal::TrivialPair(21, 26));
  ASSERT_EQ(index_to_pair<347>(), g2o::internal::TrivialPair(22, 26));
  ASSERT_EQ(index_to_pair<348>(), g2o::internal::TrivialPair(23, 26));
  ASSERT_EQ(index_to_pair<349>(), g2o::internal::TrivialPair(24, 26));
  ASSERT_EQ(index_to_pair<350>(), g2o::internal::TrivialPair(25, 26));
  ASSERT_EQ(index_to_pair<351>(), g2o::internal::TrivialPair(0, 27));
  ASSERT_EQ(index_to_pair<352>(), g2o::internal::TrivialPair(1, 27));
  ASSERT_EQ(index_to_pair<353>(), g2o::internal::TrivialPair(2, 27));
  ASSERT_EQ(index_to_pair<354>(), g2o::internal::TrivialPair(3, 27));
  ASSERT_EQ(index_to_pair<355>(), g2o::internal::TrivialPair(4, 27));
  ASSERT_EQ(index_to_pair<356>(), g2o::internal::TrivialPair(5, 27));
  ASSERT_EQ(index_to_pair<357>(), g2o::internal::TrivialPair(6, 27));
  ASSERT_EQ(index_to_pair<358>(), g2o::internal::TrivialPair(7, 27));
  ASSERT_EQ(index_to_pair<359>(), g2o::internal::TrivialPair(8, 27));
  ASSERT_EQ(index_to_pair<360>(), g2o::internal::TrivialPair(9, 27));
  ASSERT_EQ(index_to_pair<361>(), g2o::internal::TrivialPair(10, 27));
  ASSERT_EQ(index_to_pair<362>(), g2o::internal::TrivialPair(11, 27));
  ASSERT_EQ(index_to_pair<363>(), g2o::internal::TrivialPair(12, 27));
  ASSERT_EQ(index_to_pair<364>(), g2o::internal::TrivialPair(13, 27));
  ASSERT_EQ(index_to_pair<365>(), g2o::internal::TrivialPair(14, 27));
  ASSERT_EQ(index_to_pair<366>(), g2o::internal::TrivialPair(15, 27));
  ASSERT_EQ(index_to_pair<367>(), g2o::internal::TrivialPair(16, 27));
  ASSERT_EQ(index_to_pair<368>(), g2o::internal::TrivialPair(17, 27));
  ASSERT_EQ(index_to_pair<369>(), g2o::internal::TrivialPair(18, 27));
  ASSERT_EQ(index_to_pair<370>(), g2o::internal::TrivialPair(19, 27));
  ASSERT_EQ(index_to_pair<371>(), g2o::internal::TrivialPair(20, 27));
  ASSERT_EQ(index_to_pair<372>(), g2o::internal::TrivialPair(21, 27));
  ASSERT_EQ(index_to_pair<373>(), g2o::internal::TrivialPair(22, 27));
  ASSERT_EQ(index_to_pair<374>(), g2o::internal::TrivialPair(23, 27));
  ASSERT_EQ(index_to_pair<375>(), g2o::internal::TrivialPair(24, 27));
  ASSERT_EQ(index_to_pair<376>(), g2o::internal::TrivialPair(25, 27));
  ASSERT_EQ(index_to_pair<377>(), g2o::internal::TrivialPair(26, 27));
  ASSERT_EQ(index_to_pair<378>(), g2o::internal::TrivialPair(0, 28));
  ASSERT_EQ(index_to_pair<379>(), g2o::internal::TrivialPair(1, 28));
  ASSERT_EQ(index_to_pair<380>(), g2o::internal::TrivialPair(2, 28));
  ASSERT_EQ(index_to_pair<381>(), g2o::internal::TrivialPair(3, 28));
  ASSERT_EQ(index_to_pair<382>(), g2o::internal::TrivialPair(4, 28));
  ASSERT_EQ(index_to_pair<383>(), g2o::internal::TrivialPair(5, 28));
  ASSERT_EQ(index_to_pair<384>(), g2o::internal::TrivialPair(6, 28));
  ASSERT_EQ(index_to_pair<385>(), g2o::internal::TrivialPair(7, 28));
  ASSERT_EQ(index_to_pair<386>(), g2o::internal::TrivialPair(8, 28));
  ASSERT_EQ(index_to_pair<387>(), g2o::internal::TrivialPair(9, 28));
  ASSERT_EQ(index_to_pair<388>(), g2o::internal::TrivialPair(10, 28));
  ASSERT_EQ(index_to_pair<389>(), g2o::internal::TrivialPair(11, 28));
  ASSERT_EQ(index_to_pair<390>(), g2o::internal::TrivialPair(12, 28));
  ASSERT_EQ(index_to_pair<391>(), g2o::internal::TrivialPair(13, 28));
  ASSERT_EQ(index_to_pair<392>(), g2o::internal::TrivialPair(14, 28));
  ASSERT_EQ(index_to_pair<393>(), g2o::internal::TrivialPair(15, 28));
  ASSERT_EQ(index_to_pair<394>(), g2o::internal::TrivialPair(16, 28));
  ASSERT_EQ(index_to_pair<395>(), g2o::internal::TrivialPair(17, 28));
  ASSERT_EQ(index_to_pair<396>(), g2o::internal::TrivialPair(18, 28));
  ASSERT_EQ(index_to_pair<397>(), g2o::internal::TrivialPair(19, 28));
  ASSERT_EQ(index_to_pair<398>(), g2o::internal::TrivialPair(20, 28));
  ASSERT_EQ(index_to_pair<399>(), g2o::internal::TrivialPair(21, 28));
  ASSERT_EQ(index_to_pair<400>(), g2o::internal::TrivialPair(22, 28));
  ASSERT_EQ(index_to_pair<401>(), g2o::internal::TrivialPair(23, 28));
  ASSERT_EQ(index_to_pair<402>(), g2o::internal::TrivialPair(24, 28));
  ASSERT_EQ(index_to_pair<403>(), g2o::internal::TrivialPair(25, 28));
  ASSERT_EQ(index_to_pair<404>(), g2o::internal::TrivialPair(26, 28));
  ASSERT_EQ(index_to_pair<405>(), g2o::internal::TrivialPair(27, 28));
  ASSERT_EQ(index_to_pair<406>(), g2o::internal::TrivialPair(0, 29));
  ASSERT_EQ(index_to_pair<407>(), g2o::internal::TrivialPair(1, 29));
  ASSERT_EQ(index_to_pair<408>(), g2o::internal::TrivialPair(2, 29));
  ASSERT_EQ(index_to_pair<409>(), g2o::internal::TrivialPair(3, 29));
  ASSERT_EQ(index_to_pair<410>(), g2o::internal::TrivialPair(4, 29));
  ASSERT_EQ(index_to_pair<411>(), g2o::internal::TrivialPair(5, 29));
  ASSERT_EQ(index_to_pair<412>(), g2o::internal::TrivialPair(6, 29));
  ASSERT_EQ(index_to_pair<413>(), g2o::internal::TrivialPair(7, 29));
  ASSERT_EQ(index_to_pair<414>(), g2o::internal::TrivialPair(8, 29));
  ASSERT_EQ(index_to_pair<415>(), g2o::internal::TrivialPair(9, 29));
  ASSERT_EQ(index_to_pair<416>(), g2o::internal::TrivialPair(10, 29));
  ASSERT_EQ(index_to_pair<417>(), g2o::internal::TrivialPair(11, 29));
  ASSERT_EQ(index_to_pair<418>(), g2o::internal::TrivialPair(12, 29));
  ASSERT_EQ(index_to_pair<419>(), g2o::internal::TrivialPair(13, 29));
  ASSERT_EQ(index_to_pair<420>(), g2o::internal::TrivialPair(14, 29));
  ASSERT_EQ(index_to_pair<421>(), g2o::internal::TrivialPair(15, 29));
  ASSERT_EQ(index_to_pair<422>(), g2o::internal::TrivialPair(16, 29));
  ASSERT_EQ(index_to_pair<423>(), g2o::internal::TrivialPair(17, 29));
  ASSERT_EQ(index_to_pair<424>(), g2o::internal::TrivialPair(18, 29));
  ASSERT_EQ(index_to_pair<425>(), g2o::internal::TrivialPair(19, 29));
  ASSERT_EQ(index_to_pair<426>(), g2o::internal::TrivialPair(20, 29));
  ASSERT_EQ(index_to_pair<427>(), g2o::internal::TrivialPair(21, 29));
  ASSERT_EQ(index_to_pair<428>(), g2o::internal::TrivialPair(22, 29));
  ASSERT_EQ(index_to_pair<429>(), g2o::internal::TrivialPair(23, 29));
  ASSERT_EQ(index_to_pair<430>(), g2o::internal::TrivialPair(24, 29));
  ASSERT_EQ(index_to_pair<431>(), g2o::internal::TrivialPair(25, 29));
  ASSERT_EQ(index_to_pair<432>(), g2o::internal::TrivialPair(26, 29));
  ASSERT_EQ(index_to_pair<433>(), g2o::internal::TrivialPair(27, 29));
  ASSERT_EQ(index_to_pair<434>(), g2o::internal::TrivialPair(28, 29));
  ASSERT_EQ(index_to_pair<435>(), g2o::internal::TrivialPair(0, 30));
  ASSERT_EQ(index_to_pair<436>(), g2o::internal::TrivialPair(1, 30));
  ASSERT_EQ(index_to_pair<437>(), g2o::internal::TrivialPair(2, 30));
  ASSERT_EQ(index_to_pair<438>(), g2o::internal::TrivialPair(3, 30));
  ASSERT_EQ(index_to_pair<439>(), g2o::internal::TrivialPair(4, 30));
  ASSERT_EQ(index_to_pair<440>(), g2o::internal::TrivialPair(5, 30));
  ASSERT_EQ(index_to_pair<441>(), g2o::internal::TrivialPair(6, 30));
  ASSERT_EQ(index_to_pair<442>(), g2o::internal::TrivialPair(7, 30));
  ASSERT_EQ(index_to_pair<443>(), g2o::internal::TrivialPair(8, 30));
  ASSERT_EQ(index_to_pair<444>(), g2o::internal::TrivialPair(9, 30));
  ASSERT_EQ(index_to_pair<445>(), g2o::internal::TrivialPair(10, 30));
  ASSERT_EQ(index_to_pair<446>(), g2o::internal::TrivialPair(11, 30));
  ASSERT_EQ(index_to_pair<447>(), g2o::internal::TrivialPair(12, 30));
  ASSERT_EQ(index_to_pair<448>(), g2o::internal::TrivialPair(13, 30));
  ASSERT_EQ(index_to_pair<449>(), g2o::internal::TrivialPair(14, 30));
  ASSERT_EQ(index_to_pair<450>(), g2o::internal::TrivialPair(15, 30));
  ASSERT_EQ(index_to_pair<451>(), g2o::internal::TrivialPair(16, 30));
  ASSERT_EQ(index_to_pair<452>(), g2o::internal::TrivialPair(17, 30));
  ASSERT_EQ(index_to_pair<453>(), g2o::internal::TrivialPair(18, 30));
  ASSERT_EQ(index_to_pair<454>(), g2o::internal::TrivialPair(19, 30));
  ASSERT_EQ(index_to_pair<455>(), g2o::internal::TrivialPair(20, 30));
  ASSERT_EQ(index_to_pair<456>(), g2o::internal::TrivialPair(21, 30));
  ASSERT_EQ(index_to_pair<457>(), g2o::internal::TrivialPair(22, 30));
  ASSERT_EQ(index_to_pair<458>(), g2o::internal::TrivialPair(23, 30));
  ASSERT_EQ(index_to_pair<459>(), g2o::internal::TrivialPair(24, 30));
  ASSERT_EQ(index_to_pair<460>(), g2o::internal::TrivialPair(25, 30));
  ASSERT_EQ(index_to_pair<461>(), g2o::internal::TrivialPair(26, 30));
  ASSERT_EQ(index_to_pair<462>(), g2o::internal::TrivialPair(27, 30));
  ASSERT_EQ(index_to_pair<463>(), g2o::internal::TrivialPair(28, 30));
  ASSERT_EQ(index_to_pair<464>(), g2o::internal::TrivialPair(29, 30));
  ASSERT_EQ(index_to_pair<465>(), g2o::internal::TrivialPair(0, 31));
  ASSERT_EQ(index_to_pair<466>(), g2o::internal::TrivialPair(1, 31));
  ASSERT_EQ(index_to_pair<467>(), g2o::internal::TrivialPair(2, 31));
  ASSERT_EQ(index_to_pair<468>(), g2o::internal::TrivialPair(3, 31));
  ASSERT_EQ(index_to_pair<469>(), g2o::internal::TrivialPair(4, 31));
  ASSERT_EQ(index_to_pair<470>(), g2o::internal::TrivialPair(5, 31));
  ASSERT_EQ(index_to_pair<471>(), g2o::internal::TrivialPair(6, 31));
  ASSERT_EQ(index_to_pair<472>(), g2o::internal::TrivialPair(7, 31));
  ASSERT_EQ(index_to_pair<473>(), g2o::internal::TrivialPair(8, 31));
  ASSERT_EQ(index_to_pair<474>(), g2o::internal::TrivialPair(9, 31));
  ASSERT_EQ(index_to_pair<475>(), g2o::internal::TrivialPair(10, 31));
  ASSERT_EQ(index_to_pair<476>(), g2o::internal::TrivialPair(11, 31));
  ASSERT_EQ(index_to_pair<477>(), g2o::internal::TrivialPair(12, 31));
  ASSERT_EQ(index_to_pair<478>(), g2o::internal::TrivialPair(13, 31));
  ASSERT_EQ(index_to_pair<479>(), g2o::internal::TrivialPair(14, 31));
  ASSERT_EQ(index_to_pair<480>(), g2o::internal::TrivialPair(15, 31));
  ASSERT_EQ(index_to_pair<481>(), g2o::internal::TrivialPair(16, 31));
  ASSERT_EQ(index_to_pair<482>(), g2o::internal::TrivialPair(17, 31));
  ASSERT_EQ(index_to_pair<483>(), g2o::internal::TrivialPair(18, 31));
  ASSERT_EQ(index_to_pair<484>(), g2o::internal::TrivialPair(19, 31));
  ASSERT_EQ(index_to_pair<485>(), g2o::internal::TrivialPair(20, 31));
  ASSERT_EQ(index_to_pair<486>(), g2o::internal::TrivialPair(21, 31));
  ASSERT_EQ(index_to_pair<487>(), g2o::internal::TrivialPair(22, 31));
  ASSERT_EQ(index_to_pair<488>(), g2o::internal::TrivialPair(23, 31));
  ASSERT_EQ(index_to_pair<489>(), g2o::internal::TrivialPair(24, 31));
  ASSERT_EQ(index_to_pair<490>(), g2o::internal::TrivialPair(25, 31));
  ASSERT_EQ(index_to_pair<491>(), g2o::internal::TrivialPair(26, 31));
  ASSERT_EQ(index_to_pair<492>(), g2o::internal::TrivialPair(27, 31));
  ASSERT_EQ(index_to_pair<493>(), g2o::internal::TrivialPair(28, 31));
  ASSERT_EQ(index_to_pair<494>(), g2o::internal::TrivialPair(29, 31));
  ASSERT_EQ(index_to_pair<495>(), g2o::internal::TrivialPair(30, 31));
#endif

  int k = 0;
  for (int j = 0; j < 32; ++j)
    for (int i = 0; i < j; ++i) {
      ASSERT_EQ(pair_to_index(i, j), k);
      ++k;
    }
}

TEST(General, ConstantEdgeConstructor) {
  ASSERT_EQ(typeid(Edge3Dynamic::ErrorVector), typeid(Edge3Constant::ErrorVector));
  ASSERT_EQ(typeid(Edge3Dynamic::InformationType), typeid(Edge3Constant::InformationType));

  Edge3Constant e_constant;
  ASSERT_EQ(e_constant.vertices()[0], nullptr);
  ASSERT_EQ(e_constant.vertices()[1], nullptr);
  ASSERT_EQ(e_constant.vertices()[2], nullptr);
  Edge3Dynamic e_dynamic;
  ASSERT_EQ(e_dynamic.vertices()[0], e_constant.vertices()[0]);
  ASSERT_EQ(e_dynamic.vertices()[1], e_constant.vertices()[1]);
  ASSERT_EQ(e_dynamic.vertices()[2], e_constant.vertices()[2]);
}

TEST(General, FixedEdgeCreateVertex) {
  Edge3Constant e;

  auto v1 = e.createVertex(0);
  auto v2 = e.createVertex(1);
  auto v3 = e.createVertex(2);
  ASSERT_EQ(typeid(*v1), typeid(g2o::VertexSE2));
  ASSERT_EQ(typeid(*v2), typeid(g2o::VertexSE2));
  ASSERT_EQ(typeid(*v3), typeid(g2o::VertexPointXY));
  delete v3;
  delete v2;
  delete v1;

  ASSERT_EQ(nullptr, e.createVertex(-1));
  ASSERT_EQ(nullptr, e.createVertex(3));
}

template <typename EdgeType>
class EdgeTester {
 public:
  EdgeTester() {
    edge.setMeasurement(g2o::Vector2{.3, .4});
    edge.setInformation(g2o::Matrix2::Identity());

    v1.setId(0);
    v1.setEstimate(g2o::SE2(.1, .2, .3));
    v2.setId(1);
    v2.setEstimate(g2o::SE2(.3, .1, .2));
    v3.setId(2);
    v3.setEstimate(g2o::Vector2(-.3, .5));
    edge.setVertex(0, &v1);
    edge.setVertex(1, &v2);
    edge.setVertex(2, &v3);

    jacobianWorkspace.updateSize(&edge);
    jacobianWorkspace.allocate();

    hessian01.setZero();
    hessian02.setZero();
    hessian12.setZero();
    hessian00.setZero();
    hessian11.setZero();
    hessian22.setZero();
    edge.mapHessianMemory(hessian01.data(), 0, 1, false);
    edge.mapHessianMemory(hessian02.data(), 0, 2, false);
    edge.mapHessianMemory(hessian12.data(), 1, 2, false);
    v1.mapHessianMemory(hessian00.data());
    v2.mapHessianMemory(hessian11.data());
    v3.mapHessianMemory(hessian22.data());
  }

  EdgeType edge;

  g2o::VertexSE2 v1;
  g2o::VertexSE2 v2;
  g2o::VertexPointXY v3;

  g2o::JacobianWorkspace jacobianWorkspace;

  Eigen::Matrix<number_t, 3, 3> hessian01;
  Eigen::Matrix<number_t, 3, 2> hessian02;
  Eigen::Matrix<number_t, 3, 2> hessian12;
  Eigen::Matrix<number_t, 3, 3> hessian00;
  Eigen::Matrix<number_t, 3, 3> hessian11;
  Eigen::Matrix<number_t, 2, 2> hessian22;
};

TEST(ConstantEdgeTest, ConstantEdge_allVerticesFixed) {
  EdgeTester<Edge3Dynamic> dynamic;
  EdgeTester<Edge3Constant> constant;
  ASSERT_EQ(dynamic.edge.allVerticesFixed(), constant.edge.allVerticesFixed());
  ASSERT_FALSE(constant.edge.allVerticesFixed());
  dynamic.v1.setFixed(true);
  dynamic.v2.setFixed(true);
  dynamic.v3.setFixed(true);
  constant.v1.setFixed(true);
  constant.v2.setFixed(true);
  constant.v3.setFixed(true);
  ASSERT_EQ(dynamic.edge.allVerticesFixed(), constant.edge.allVerticesFixed());
  ASSERT_TRUE(constant.edge.allVerticesFixed());
}

TEST(ConstantEdgeTest, ConstantEdge_computeError) {
  EdgeTester<Edge3Dynamic> dynamic;
  EdgeTester<Edge3Constant> constant;
  dynamic.edge.computeError();
  constant.edge.computeError();
  ASSERT_DOUBLE_EQ(0.0, (dynamic.edge.error() - constant.edge.error()).norm());
}

TEST(ConstantEdgeTest, ConstantEdge_linearizeOplus) {
  EdgeTester<Edge3Dynamic> dynamic;
  EdgeTester<Edge3Constant> constant;
  dynamic.edge.computeError();
  constant.edge.computeError();
  dynamic.edge.linearizeOplus(dynamic.jacobianWorkspace);
  constant.edge.linearizeOplus(constant.jacobianWorkspace);
  ASSERT_DOUBLE_EQ(
      0.0, (Eigen::Map<g2o::MatrixX>(dynamic.jacobianWorkspace.workspaceForVertex(0), 2, 3) -
            Eigen::Map<g2o::MatrixX>(constant.jacobianWorkspace.workspaceForVertex(0), 2, 3))
               .norm());
  ASSERT_DOUBLE_EQ(
      0.0, (Eigen::Map<g2o::MatrixX>(dynamic.jacobianWorkspace.workspaceForVertex(1), 2, 3) -
            Eigen::Map<g2o::MatrixX>(constant.jacobianWorkspace.workspaceForVertex(1), 2, 3))
               .norm());
  ASSERT_DOUBLE_EQ(
      0.0, (Eigen::Map<g2o::MatrixX>(dynamic.jacobianWorkspace.workspaceForVertex(2), 2, 2) -
            Eigen::Map<g2o::MatrixX>(constant.jacobianWorkspace.workspaceForVertex(2), 2, 2))
               .norm());
}

TEST(ConstantEdgeTest, ConstantEdge_constructQuadraticForm) {
  EdgeTester<Edge3Dynamic> dynamic;
  EdgeTester<Edge3Constant> constant;
  dynamic.edge.computeError();
  constant.edge.computeError();
  dynamic.edge.linearizeOplus(dynamic.jacobianWorkspace);
  constant.edge.linearizeOplus(constant.jacobianWorkspace);

  dynamic.edge.constructQuadraticForm();
  constant.edge.constructQuadraticForm();

  ASSERT_DOUBLE_EQ(0.0, (dynamic.hessian00 - constant.hessian00).norm());
  ASSERT_DOUBLE_EQ(0.0, (dynamic.hessian11 - constant.hessian11).norm());
  ASSERT_DOUBLE_EQ(0.0, (dynamic.hessian22 - constant.hessian22).norm());
  ASSERT_DOUBLE_EQ(0.0, (dynamic.hessian01 - constant.hessian01).norm());
  ASSERT_DOUBLE_EQ(0.0, (dynamic.hessian02 - constant.hessian02).norm());
  ASSERT_DOUBLE_EQ(0.0, (dynamic.hessian12 - constant.hessian12).norm());
}

TEST(ConstantEdgeTest, ConstantEdge_constructQuadraticForm_robust) {
  EdgeTester<Edge3Dynamic> dynamic;
  EdgeTester<Edge3Constant> constant;

  dynamic.edge.setMeasurement(g2o::Vector2{.3, 3.4});
  constant.edge.setMeasurement(g2o::Vector2{.3, 3.4});

  // assuming that G2O_NO_IMPLICIT_OWNERSHIP_OF_OBJECTS is false
  g2o::RobustKernelHuber* rk_dynamic = new g2o::RobustKernelHuber;
  dynamic.edge.setRobustKernel(rk_dynamic);
  dynamic.edge.computeError();
  dynamic.edge.linearizeOplus(dynamic.jacobianWorkspace);
  dynamic.edge.constructQuadraticForm();

  g2o::RobustKernelHuber* rk_constant = new g2o::RobustKernelHuber;
  constant.edge.setRobustKernel(rk_constant);
  constant.edge.computeError();
  constant.edge.linearizeOplus(constant.jacobianWorkspace);
  constant.edge.constructQuadraticForm();
  ASSERT_EQ(true, (dynamic.edge.error() - constant.edge.error()).norm() < 1e-7);
  ASSERT_DOUBLE_EQ(
      0.0, (Eigen::Map<g2o::MatrixX>(dynamic.jacobianWorkspace.workspaceForVertex(0), 2, 3) -
            Eigen::Map<g2o::MatrixX>(constant.jacobianWorkspace.workspaceForVertex(0), 2, 3))
               .norm());
  ASSERT_DOUBLE_EQ(0.0, (dynamic.hessian00 - constant.hessian00).norm());
  ASSERT_DOUBLE_EQ(0.0, (dynamic.hessian11 - constant.hessian11).norm());
  ASSERT_DOUBLE_EQ(0.0, (dynamic.hessian22 - constant.hessian22).norm());
  ASSERT_DOUBLE_EQ(0.0, (dynamic.hessian01 - constant.hessian01).norm());
  ASSERT_DOUBLE_EQ(0.0, (dynamic.hessian02 - constant.hessian02).norm());
  ASSERT_DOUBLE_EQ(0.0, (dynamic.hessian12 - constant.hessian12).norm());
}

TEST(ConstantEdgeTest, ConstantEdge_constructQuadraticForm_rowMajor) {
  EdgeTester<Edge3Dynamic> dynamic;
  EdgeTester<Edge3Constant> constant;
  dynamic.edge.mapHessianMemory(dynamic.hessian01.data(), 0, 1, true);
  Eigen::Matrix<number_t, 2, 3> hessian20_dynamic;
  Eigen::Matrix<number_t, 2, 3> hessian21_dynamic;
  hessian20_dynamic.setZero();
  hessian21_dynamic.setZero();
  dynamic.edge.mapHessianMemory(hessian20_dynamic.data(), 0, 2, true);
  dynamic.edge.mapHessianMemory(hessian21_dynamic.data(), 1, 2, true);

  constant.edge.mapHessianMemory(constant.hessian01.data(), 0, 1, true);
  Eigen::Matrix<number_t, 2, 3> hessian20_constant;
  Eigen::Matrix<number_t, 2, 3> hessian21_constant;
  hessian20_constant.setZero();
  hessian21_constant.setZero();
  constant.edge.mapHessianMemory(hessian20_constant.data(), 0, 2, true);
  constant.edge.mapHessianMemory(hessian21_constant.data(), 1, 2, true);

  dynamic.edge.computeError();
  constant.edge.computeError();
  dynamic.edge.linearizeOplus(dynamic.jacobianWorkspace);
  constant.edge.linearizeOplus(constant.jacobianWorkspace);

  dynamic.edge.constructQuadraticForm();
  constant.edge.constructQuadraticForm();

  ASSERT_DOUBLE_EQ(0.0, (dynamic.hessian00 - constant.hessian00).norm());
  ASSERT_DOUBLE_EQ(0.0, (dynamic.hessian11 - constant.hessian11).norm());
  ASSERT_DOUBLE_EQ(0.0, (dynamic.hessian22 - constant.hessian22).norm());
  ASSERT_DOUBLE_EQ(0.0, (dynamic.hessian01 - constant.hessian01).norm());
  ASSERT_DOUBLE_EQ(0.0, (hessian20_dynamic - hessian20_constant).norm());
  ASSERT_DOUBLE_EQ(0.0, (hessian21_dynamic - hessian21_constant).norm());

  EdgeTester<Edge3Constant> constant_colMajor;
  constant_colMajor.edge.computeError();
  constant_colMajor.edge.linearizeOplus(constant_colMajor.jacobianWorkspace);
  constant_colMajor.edge.constructQuadraticForm();
  ASSERT_DOUBLE_EQ(0.0, (constant_colMajor.hessian01 - constant.hessian01.transpose()).norm());
  ASSERT_DOUBLE_EQ(0.0, (constant_colMajor.hessian02 - hessian20_constant.transpose()).norm());
  ASSERT_DOUBLE_EQ(0.0, (constant_colMajor.hessian12 - hessian21_constant.transpose()).norm());
}
