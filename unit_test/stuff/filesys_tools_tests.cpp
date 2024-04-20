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

#include <filesystem>
#include <regex>

#include "g2o/config.h"
#include "g2o/stuff/filesys_tools.h"
#include "gmock/gmock.h"

TEST(Stuff, GetFileExtension) {
  EXPECT_EQ("txt", g2o::getFileExtension("test.txt"));
  EXPECT_EQ("txt", g2o::getFileExtension("/home/g2o/test.txt"));
  EXPECT_EQ("", g2o::getFileExtension("/home/g2o/test"));
  EXPECT_EQ("", g2o::getFileExtension(""));
}

TEST(Stuff, GetPureFilename) {
  EXPECT_EQ("test", g2o::getPureFilename("test.txt"));
  EXPECT_EQ("test", g2o::getPureFilename("test"));
  EXPECT_EQ("/home/g2o/test", g2o::getPureFilename("/home/g2o/test.txt"));
  EXPECT_EQ("", g2o::getPureFilename(""));
}

TEST(Stuff, ChangeFileExtension) {
  EXPECT_EQ("test.dat", g2o::changeFileExtension("test.txt", "dat"));
  EXPECT_EQ("test.dat", g2o::changeFileExtension("test.txt", ".dat"));
  EXPECT_EQ("test", g2o::changeFileExtension("test", "dat"));
  EXPECT_EQ("test", g2o::changeFileExtension("test", ".dat"));
}

TEST(Stuff, GetBasename) {
  EXPECT_EQ("test.txt", g2o::getBasename("test.txt"));
  EXPECT_EQ("test", g2o::getBasename("test"));
#ifdef WINDOWS
  EXPECT_EQ("test.txt", g2o::getBasename("C:\\users\\g2o\\test.txt"));
#else
  EXPECT_EQ("test.txt", g2o::getBasename("/home/g2o/test.txt"));
#endif
  EXPECT_EQ("", g2o::getBasename(""));
}

TEST(Stuff, GetDirname) {
  EXPECT_EQ("", g2o::getDirname("test.txt"));
  EXPECT_EQ("", g2o::getDirname("test"));
#ifdef WINDOWS
  EXPECT_EQ("C:\\users\\g2o", g2o::getDirname("C:\\users\\g2o\\test.txt"));
#else
  EXPECT_EQ("/home/g2o", g2o::getDirname("/home/g2o/test.txt"));
#endif
  EXPECT_EQ("", g2o::getDirname(""));
}

TEST(Stuff, FileExists) {
  namespace fs = std::filesystem;

  EXPECT_FALSE(g2o::fileExists("test12345.txt"));
  EXPECT_FALSE(g2o::fileExists("test12345"));

  EXPECT_TRUE(g2o::fileExists(G2O_SRC_DIR));
  EXPECT_TRUE(g2o::fileExists(
      (fs::path(G2O_SRC_DIR) / fs::path("CMakeLists.txt")).string()));
  EXPECT_FALSE(g2o::fileExists(G2O_SRC_DIR, true));
  EXPECT_TRUE(g2o::fileExists(
      (fs::path(G2O_SRC_DIR) / fs::path("CMakeLists.txt")).string(), true));
}

TEST(Stuff, GetFilesByPattern) {
  using namespace testing;
  namespace fs = std::filesystem;
  const std::string directory =
      (fs::path(G2O_SRC_DIR) / fs::path("doc")).string();
  const std::regex pattern("^license.*\\.txt$");
  const std::vector<std::string> licenseFiles =
      g2o::getFilesByPattern(directory, pattern);
  EXPECT_THAT(licenseFiles, SizeIs(3));
  EXPECT_THAT(licenseFiles, Each(ContainsRegex("license.*\\.txt$")));
}
