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

#include "g2o/config.h"
#include "g2o/stuff/filesys_tools.h"
#include "gmock/gmock.h"

namespace {
#ifdef WINDOWS
static const std::string pathSep = "\\";
#else
static const std::string pathSep = "/";
#endif
}  // namespace

TEST(Stuff, GetFileExtension) {
  ASSERT_EQ("txt", g2o::getFileExtension("test.txt"));
  ASSERT_EQ("txt", g2o::getFileExtension("/home/g2o/test.txt"));
  ASSERT_EQ("", g2o::getFileExtension("/home/g2o/test"));
  ASSERT_EQ("", g2o::getFileExtension(""));
}

TEST(Stuff, GetPureFilename) {
  ASSERT_EQ("test", g2o::getPureFilename("test.txt"));
  ASSERT_EQ("test", g2o::getPureFilename("test"));
  ASSERT_EQ("/home/g2o/test", g2o::getPureFilename("/home/g2o/test.txt"));
  ASSERT_EQ("", g2o::getPureFilename(""));
}

TEST(Stuff, ChangeFileExtension) {
  ASSERT_EQ("test.dat", g2o::changeFileExtension("test.txt", "dat", false));
  ASSERT_EQ("test.dat", g2o::changeFileExtension("test.txt", ".dat", true));
  ASSERT_EQ("test", g2o::changeFileExtension("test", "dat", false));
  ASSERT_EQ("test", g2o::changeFileExtension("test", ".dat", true));
}

TEST(Stuff, GetBasename) {
  ASSERT_EQ("test.txt", g2o::getBasename("test.txt"));
  ASSERT_EQ("test", g2o::getBasename("test"));
#ifdef WINDOWS
  ASSERT_EQ("test.txt", g2o::getBasename("C:\\users\\g2o\\test.txt"));
#else
  ASSERT_EQ("test.txt", g2o::getBasename("/home/g2o/test.txt"));
#endif
  ASSERT_EQ("", g2o::getBasename(""));
}

TEST(Stuff, GetDirname) {
  ASSERT_EQ("", g2o::getDirname("test.txt"));
  ASSERT_EQ("", g2o::getDirname("test"));
#ifdef WINDOWS
  ASSERT_EQ("C:\\users\\g2o", g2o::getDirname("C:\\users\\g2o\\test.txt"));
#else
  ASSERT_EQ("/home/g2o", g2o::getDirname("/home/g2o/test.txt"));
#endif
  ASSERT_EQ("", g2o::getDirname(""));
}

TEST(Stuff, FileExists) {
  ASSERT_FALSE(g2o::fileExists("test12345.txt"));
  ASSERT_FALSE(g2o::fileExists("test12345"));

  ASSERT_TRUE(g2o::fileExists(G2O_SRC_DIR));
  ASSERT_TRUE(g2o::fileExists(
      (std::string(G2O_SRC_DIR) + pathSep + "CMakeLists.txt").c_str()));
}

TEST(Stuff, GetFilesByPattern) {
  using namespace testing;
  std::string pattern =
      std::string(G2O_SRC_DIR) + pathSep + "doc" + pathSep + "license*.txt";
  std::vector<std::string> licenseFiles =
      g2o::getFilesByPattern(pattern.c_str());
  ASSERT_THAT(licenseFiles, SizeIs(3));
  ASSERT_THAT(licenseFiles, Each(ContainsRegex("license.*\\.txt$")));
}
