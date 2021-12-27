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

#include <cstdlib>

#include "g2o/stuff/string_tools.h"
#include "gtest/gtest.h"

TEST(Stuff, Trim) {
  ASSERT_EQ("abc", g2o::trim("abc   "));
  ASSERT_EQ("abc", g2o::trim("   abc"));
  ASSERT_EQ("abc", g2o::trim("   abc   "));
  ASSERT_EQ("abc", g2o::trim("abc"));
  ASSERT_EQ("", g2o::trim("    "));
  ASSERT_EQ("abc   ", g2o::trimLeft("abc   "));
  ASSERT_EQ("abc", g2o::trimLeft("   abc"));
  ASSERT_EQ("abc   ", g2o::trimLeft("   abc   "));
  ASSERT_EQ("abc", g2o::trimLeft("abc"));
  ASSERT_EQ("", g2o::trimLeft("    "));
  ASSERT_EQ("abc", g2o::trimRight("abc   "));
  ASSERT_EQ("   abc", g2o::trimRight("   abc"));
  ASSERT_EQ("   abc", g2o::trimRight("   abc   "));
  ASSERT_EQ("abc", g2o::trimRight("abc"));
  ASSERT_EQ("", g2o::trimRight("    "));
}

TEST(Stuff, StrToLower) {
  ASSERT_EQ("abc", g2o::strToLower("abc"));
  ASSERT_EQ("abc", g2o::strToLower("Abc"));
  ASSERT_EQ("abc", g2o::strToLower("AbC"));
  ASSERT_EQ("abc", g2o::strToLower("ABC"));
  ASSERT_EQ("abc !!$", g2o::strToLower("ABC !!$"));
}

TEST(Stuff, StrToUpper) {
  ASSERT_EQ("ABC", g2o::strToUpper("abc"));
  ASSERT_EQ("ABC", g2o::strToUpper("Abc"));
  ASSERT_EQ("ABC", g2o::strToUpper("AbC"));
  ASSERT_EQ("ABC", g2o::strToUpper("ABC"));
  ASSERT_EQ("ABC !!$", g2o::strToUpper("ABC !!$"));
}

TEST(Stuff, FormatString) {
  ASSERT_EQ("42", g2o::formatString("%d", 42));
  ASSERT_EQ("3.1415", g2o::formatString("%g", 3.1415));
  ASSERT_EQ("3.141500", g2o::formatString("%f", 3.1415));
  ASSERT_EQ("3.142", g2o::formatString("%.3f", 3.1415));
  ASSERT_EQ("Hello 42 World", g2o::formatString("Hello %d World", 42));
}

TEST(Stuff, StrSplit) {
  std::vector<std::string> tokens;
  tokens = g2o::strSplit("", ",");
  ASSERT_EQ(0, tokens.size());
  tokens = g2o::strSplit("42", ",");
  ASSERT_EQ(1, tokens.size());
  ASSERT_EQ("42", tokens[0]);
  tokens = g2o::strSplit("1,2;3:4", ";,:");
  ASSERT_EQ(4, tokens.size());
  for (size_t i = 0; i < tokens.size(); ++i)
    ASSERT_EQ(g2o::formatString("%d", int(i + 1)), tokens[i]);
}

TEST(Stuff, StrStartsWith) {
  ASSERT_FALSE(g2o::strStartsWith("Hello World!", "World!"));
  ASSERT_TRUE(g2o::strStartsWith("Hello World!", "Hello"));
  ASSERT_TRUE(g2o::strStartsWith("Hello World!", "Hello World!"));
  ASSERT_FALSE(g2o::strStartsWith("Hello World!", "Hello World!!!"));
}

TEST(Stuff, StrEndsWith) {
  ASSERT_TRUE(g2o::strEndsWith("Hello World!", "World!"));
  ASSERT_FALSE(g2o::strEndsWith("Hello World!", "Hello"));
  ASSERT_TRUE(g2o::strEndsWith("Hello World!", "Hello World!"));
  ASSERT_FALSE(g2o::strEndsWith("Hello World!", "!!Hello World!"));
}

#if defined(UNIX) && !defined(ANDROID)
TEST(Stuff, StrExpand) {
  char* envVar = getenv("HOME");
  if (envVar == nullptr) {
    std::cerr << "HOME not defined" << std::endl;
    SUCCEED();
    return;
  }
  std::string expanded = g2o::strExpandFilename("$HOME/filename.txt");
  std::string expected = g2o::formatString("%s/filename.txt", envVar);
  EXPECT_EQ(expanded, expected);
}
#endif
