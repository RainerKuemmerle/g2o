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

#include "g2o/stuff/property.h"
#include "gtest/gtest.h"

class PropertyTest : public ::testing::Test {
 protected:
  void SetUp() override {
    propertyMap_.makeProperty<g2o::IntProperty>("int", 42);
    propertyMap_.makeProperty<g2o::DoubleProperty>("double", 133.7);
    propertyMap_.makeProperty<g2o::StringProperty>("string", "g2o");
  }

  g2o::PropertyMap propertyMap_;
};

TEST_F(PropertyTest, Erase) {
  EXPECT_EQ(propertyMap_.size(), 3);
  EXPECT_TRUE(propertyMap_.getProperty<g2o::StringProperty>("string"));
  const bool erased = propertyMap_.eraseProperty("string");
  EXPECT_TRUE(erased);
  EXPECT_EQ(propertyMap_.size(), 2);
  EXPECT_FALSE(propertyMap_.getProperty<g2o::StringProperty>("string"));
}

TEST_F(PropertyTest, Add) {
  EXPECT_EQ(propertyMap_.size(), 3);
  auto property = std::make_shared<g2o::IntProperty>("new_prop", 81);
  propertyMap_.addProperty(property);
  EXPECT_EQ(propertyMap_.size(), 4);
  EXPECT_EQ(propertyMap_.getProperty<g2o::IntProperty>("new_prop"), property);
}

TEST_F(PropertyTest, UpdatePropertyFromString) {
  bool updated = propertyMap_.updatePropertyFromString("int", "21");
  EXPECT_TRUE(updated);
  auto property = propertyMap_.getProperty<g2o::IntProperty>("int");
  EXPECT_EQ(property->value(), 21);
  updated = propertyMap_.updatePropertyFromString("not_in", "42");
  EXPECT_FALSE(updated);
}

TEST_F(PropertyTest, UpdateMapFromString) {
  auto intProperty = propertyMap_.getProperty<g2o::IntProperty>("int");
  auto stringProperty = propertyMap_.getProperty<g2o::StringProperty>("string");
  const bool updated = propertyMap_.updateMapFromString("int=21,string=updated");
  EXPECT_TRUE(updated);
  EXPECT_EQ(intProperty->value(), 21);
  EXPECT_EQ(stringProperty->value(), "updated");
}

TEST_F(PropertyTest, WriteCSV) {
  std::stringstream output;
  propertyMap_.writeToCSV(output);
  std::string lines[2];
  std::getline(output, lines[0]);
  std::getline(output, lines[1]);
  EXPECT_EQ(lines[0], "double,int,string");
  EXPECT_EQ(lines[1], "133.7,42,g2o");
}
