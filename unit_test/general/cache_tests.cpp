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

#include <memory>

#include "g2o/core/cache.h"
#include "g2o/core/optimizable_graph.h"

using namespace testing;  // NOLINT

namespace {

/**
 * Mock cache implementation for testing
 */
class MockCache : public g2o::Cache {
 public:
  MockCache() = default;
  virtual ~MockCache() = default;

  int updateCount = 0;
  using g2o::Cache::updateNeeded_;  // make accessible for testing

 protected:
  void updateImpl() override {
    // Simple mock implementation
    updateCount++;
  }
};

}  // namespace

/**
 * Test suite for Cache::CacheKey
 */
class CacheKeyTest : public Test {};

TEST_F(CacheKeyTest, DefaultConstruction) {
  g2o::Cache::CacheKey key;
  EXPECT_TRUE(key.type().empty());
  EXPECT_TRUE(key.parameters().empty());
}

TEST_F(CacheKeyTest, ConstructionWithParameters) {
  g2o::ParameterVector params;
  g2o::Cache::CacheKey key("test_type", params);
  EXPECT_EQ("test_type", key.type());
  EXPECT_EQ(params, key.parameters());
}

TEST_F(CacheKeyTest, ComparisonOperator) {
  g2o::ParameterVector params1;
  g2o::ParameterVector params2;
  g2o::ParameterVector params3;

  g2o::Cache::CacheKey key1("type_a", params1);
  g2o::Cache::CacheKey key2("type_b", params2);
  g2o::Cache::CacheKey key3("type_a", params3);

  // Different types should be compared
  EXPECT_TRUE(key1 < key2);
  EXPECT_FALSE(key2 < key1);

  // Same keys should not satisfy <
  EXPECT_FALSE(key1 < key3);
  EXPECT_FALSE(key3 < key1);
}

/**
 * Test suite for Cache base class functionality
 */
class CacheTest : public Test {
 protected:
  std::unique_ptr<g2o::OptimizableGraph> graph_;

  void SetUp() override { graph_ = std::make_unique<g2o::OptimizableGraph>(); }
};

TEST_F(CacheTest, ElementType) {
  auto cache = std::make_shared<MockCache>();
  EXPECT_EQ(g2o::HyperGraph::kHgetCache, cache->elementType());
}

TEST_F(CacheTest, UpdateNeeded) {
  auto cache = std::make_shared<MockCache>();

  // Cache should be marked as needing update initially
  EXPECT_TRUE(cache->updateNeeded_);
}

TEST_F(CacheTest, UpdateMethod) {
  auto cache = std::make_shared<MockCache>();

  // Call update multiple times
  cache->update();
  EXPECT_EQ(1, cache->updateCount);
  EXPECT_FALSE(cache->updateNeeded_);

  cache->update();
  EXPECT_EQ(1, cache->updateCount);
  EXPECT_FALSE(cache->updateNeeded_);

  // Mark as needing update again
  cache->updateNeeded_ = true;
  cache->update();
  EXPECT_EQ(2, cache->updateCount);
  EXPECT_FALSE(cache->updateNeeded_);
}
