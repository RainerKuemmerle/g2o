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

#include "io_format.h"

#include <optional>
#include <string_view>

namespace g2o::io {

std::string_view to_string(g2o::io::Format format) {
  switch (format) {
    case g2o::io::Format::kUndefined:
      return "Undefined";
    case g2o::io::Format::kG2O:
      return "G2O";
    case g2o::io::Format::kBinary:
      return "Binary";
    case g2o::io::Format::kJson:
      return "JSON";
  }
  return "";
}

std::optional<Format> formatForFileExtension(std::string_view extension) {
  if (extension == "g2o" || extension == "G2O") return Format::kG2O;
  if (extension == "json" || extension == "JSON") return Format::kJson;
  if (extension == "bin" || extension == "BIN") return Format::kBinary;
  return std::nullopt;
}

FileFilter::FileFilter(std::string filter, Format format)
    : filter(std::move(filter)), format(format) {}

bool FileFilter::operator==(const FileFilter& other) const {
  return filter == other.filter && format == other.format;
}

std::vector<FileFilter> getFileFilter(bool open) {
  std::vector<FileFilter> result;
  result.emplace_back("g2o Ascii files (*.g2o)", Format::kG2O);
  result.emplace_back("g2o Json files (*.json)", Format::kJson);
  result.emplace_back("g2o BIN files (*.bin)", Format::kBinary);
  if (open) {
    result.emplace_back("All Files (*)", Format::kUndefined);
  }
  return result;
}

}  // namespace g2o::io
