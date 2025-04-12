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

#include "filesys_tools.h"

#include <filesystem>
#include <regex>
#include <string_view>

namespace g2o {

std::string getFileExtension(std::string_view filename) {
  const std::filesystem::path path(filename);
  const std::string result = path.extension().string();
  if (!result.empty() && result.front() == '.') return result.substr(1);
  return result;
}

std::string getPureFilename(std::string_view filename) {
  const std::filesystem::path path(filename);
  if (path.has_parent_path()) {
    return (path.parent_path() / path.stem()).string();
  }
  return path.stem().string();
}

std::string getBasename(std::string_view filename) {
  const std::filesystem::path path(filename);
  return path.filename().string();
}

std::string getDirname(std::string_view filename) {
  const std::filesystem::path path(filename);
  return path.parent_path().string();
}

std::string changeFileExtension(std::string_view filename,
                                std::string_view newExt) {
  std::filesystem::path path(filename);
  if (!path.has_extension()) {
    return path.string();
  }
  path.replace_extension(std::filesystem::path(newExt));
  return path.string();
}

bool fileExists(std::string_view filename, bool regular) {
  const std::filesystem::path path(filename);
  return std::filesystem::exists(path) &&
         (!regular || std::filesystem::is_regular_file(path));
}

std::vector<std::string> getFilesByPattern(std::string_view directory,
                                           const std::regex& pattern) {
  auto match = [&pattern](const std::filesystem::path& entry) {
    return std::regex_search(entry.filename().string(), pattern);
  };

  std::vector<std::string> result;
  for (const auto& dir_entry :
       std::filesystem::directory_iterator(std::filesystem::path(directory))) {
    if (dir_entry.is_regular_file() && match(dir_entry)) {
      result.emplace_back(dir_entry.path().string());
      continue;
    }
    if (dir_entry.is_symlink() &&
        std::filesystem::is_regular_file(
            std::filesystem::read_symlink(dir_entry)) &&
        match(dir_entry)) {
      result.emplace_back(dir_entry.path().string());
      continue;
    }
  }

  return result;
}

}  // namespace g2o
