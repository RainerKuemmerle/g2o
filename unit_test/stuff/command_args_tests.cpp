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

#include <vector>

#include "g2o/stuff/command_args.h"
#include "gtest/gtest.h"

class CommandArgsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    arg.param(argumentNames[0], valueInt, 5, "integer value");
    arg.param(argumentNames[1], valueDouble, 1e-6, "double value");
    arg.param(argumentNames[2], valueBool, false, "boolean");
    arg.param(argumentNames[3], valueVectorInt, std::vector<int>(), "vector of int");
    arg.param(argumentNames[4], valueVectorDouble, std::vector<double>(), "vector of double");
    arg.param(argumentNames[5], valueString, "", "string value");
    arg.paramLeftOver("input", valueStringLeftOverOptional, "", "optional param", true);
  }

  void call(const std::vector<std::string> &arguments)
  {
    std::string testname = "unit_test";
    int argc = arguments.size() + 1;
    char** argv = new char*[argc];
    argv[0] = const_cast<char*>(testname.c_str());
    for (size_t i = 0; i < arguments.size(); ++i) {
      argv[i+1] = const_cast<char*>(arguments[i].c_str());
    }
    arg.parseArgs(argc, argv, false);
    delete[] argv;
  }

  g2o::CommandArgs arg;

  std::vector<std::string> argumentNames = {"i", "d", "b", "vi", "vd", "string"};

  int valueInt;
  double valueDouble;
  bool valueBool;
  std::string valueString;
  std::vector<int> valueVectorInt;
  std::vector<double> valueVectorDouble;
  std::string valueStringLeftOverOptional;
};

TEST_F(CommandArgsTest, DefaultValues) {
  std::vector<std::string> arguments;
  call(arguments);
  for (const auto& p : argumentNames) {
    EXPECT_FALSE(arg.parsedParam(p));
  }
  EXPECT_EQ(valueInt, 5);
  EXPECT_DOUBLE_EQ(valueDouble, 1e-6);
  EXPECT_FALSE(valueBool);
  EXPECT_EQ(valueVectorInt.size(), 0);
  EXPECT_EQ(valueVectorDouble.size(), 0);
  EXPECT_EQ(valueString, "");
  EXPECT_EQ(valueStringLeftOverOptional, "");
}

TEST_F(CommandArgsTest, ParseValues) {
  std::vector<std::string> arguments = {
    "-" + argumentNames[0], "42",
    "-" + argumentNames[1], "0.5",
    "-" + argumentNames[2],
    "-" + argumentNames[3], "1,2,3,4",
    "-" + argumentNames[4], "1.,20.,3.4",
    "-" + argumentNames[5], "summary.txt",
    "optional.txt"
  };
  call(arguments);
  for (const auto& p : argumentNames) {
    EXPECT_TRUE(arg.parsedParam(p)) << "failed param: " << p;
  }
  EXPECT_EQ(valueInt, 42);
  EXPECT_DOUBLE_EQ(valueDouble, 0.5);
  EXPECT_TRUE(valueBool);
  EXPECT_EQ(valueVectorInt, std::vector<int>({1,2,3,4}));
  EXPECT_EQ(valueVectorDouble, std::vector<double>({1.,20.,3.4}));
  EXPECT_EQ(valueString, "summary.txt");
  EXPECT_EQ(valueStringLeftOverOptional, "optional.txt");
}
