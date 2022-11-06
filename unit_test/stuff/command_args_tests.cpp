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

#include <algorithm>
#include <vector>

#include "g2o/stuff/command_args.h"
#include "gtest/gtest.h"

class CommandArgsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    arg_.param(argumentNames_[0], valueInt_, 5, "integer value");
    arg_.param(argumentNames_[1], valueDouble_, 1e-6, "double value");
    arg_.param(argumentNames_[2], valueBool_, false, "boolean");
    arg_.param(argumentNames_[3], valueVectorInt_, std::vector<int>(),
              "vector of int");
    arg_.param(argumentNames_[4], valueVectorDouble_, std::vector<double>(),
              "vector of double");
    arg_.param(argumentNames_[5], valueString_, "", "string value");
    arg_.param(argumentNames_[6], valueFloat_, 1e4, "float value");
    arg_.paramLeftOver("input", valueStringLeftOverOptional_, "",
                      "optional param", true);
  }

  void call(const std::vector<std::string>& arguments) {
    const std::string testname = "unit_test";
    const int argc = arguments.size() + 1;
    char** argv = new char*[argc];
    for (int i = 0; i < argc; ++i) argv[i] = nullptr;
    argv[0] = copyIntoArray(testname);
    for (size_t i = 0; i < arguments.size(); ++i) {
      argv[i + 1] = copyIntoArray(arguments[i]);
    }
    arg_.parseArgs(argc, argv, false);
    for (int i = 0; i < argc; ++i) delete[] argv[i];
    delete[] argv;
  }

  static char* copyIntoArray(const std::string& s) {
    char* cstr = new char[s.size() + 1];
    std::copy(s.begin(), s.end(), cstr);
    cstr[s.size()] = '\0';
    return cstr;
  }

  g2o::CommandArgs arg_;

  std::vector<std::string> argumentNames_ = {"i",  "d",      "b", "vi",
                                            "vd", "string", "f"};

  int valueInt_;
  double valueDouble_;
  float valueFloat_;
  bool valueBool_;
  std::string valueString_;
  std::vector<int> valueVectorInt_;
  std::vector<double> valueVectorDouble_;
  std::string valueStringLeftOverOptional_;
};

TEST_F(CommandArgsTest, DefaultValues) {
  const std::vector<std::string> arguments;
  call(arguments);
  for (const auto& p : argumentNames_) {
    EXPECT_FALSE(arg_.parsedParam(p));
  }
  EXPECT_EQ(valueInt_, 5);
  EXPECT_DOUBLE_EQ(valueDouble_, 1e-6);
  EXPECT_FLOAT_EQ(valueFloat_, 1e4);
  EXPECT_FALSE(valueBool_);
  EXPECT_EQ(valueVectorInt_.size(), 0);
  EXPECT_EQ(valueVectorDouble_.size(), 0);
  EXPECT_EQ(valueString_, "");
  EXPECT_EQ(valueStringLeftOverOptional_, "");
}

TEST_F(CommandArgsTest, ParseValues) {
  // clang-format off
  const std::vector<std::string> arguments = {
    "-" + argumentNames_[0], "42",
    "-" + argumentNames_[1], "0.5",
    "-" + argumentNames_[2],
    "-" + argumentNames_[3], "1,2,3,4",
    "-" + argumentNames_[4], "1.,20.,3.4",
    "-" + argumentNames_[5], "summary.txt",
    "-" + argumentNames_[6], "17.4",
    "optional.txt"
  };
  // clang-format on
  call(arguments);
  for (const auto& p : argumentNames_) {
    EXPECT_TRUE(arg_.parsedParam(p)) << "failed param: " << p;
  }
  EXPECT_EQ(valueInt_, 42);
  EXPECT_DOUBLE_EQ(valueDouble_, 0.5);
  EXPECT_FLOAT_EQ(valueFloat_, 17.4);
  EXPECT_TRUE(valueBool_);
  EXPECT_EQ(valueVectorInt_, std::vector<int>({1, 2, 3, 4}));
  EXPECT_EQ(valueVectorDouble_, std::vector<double>({1., 20., 3.4}));
  EXPECT_EQ(valueString_, "summary.txt");
  EXPECT_EQ(valueStringLeftOverOptional_, "optional.txt");
}
