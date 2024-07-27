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

#ifndef G2O_CORE_IO_FORMAT_H
#define G2O_CORE_IO_FORMAT_H

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "g2o/core/g2o_core_api.h"
namespace g2o::io {

enum class G2O_CORE_API Format {
  kUndefined = -1,
  kG2O = 0,
  kBinary = 1,
  kJson = 2,
  kXML = 3
};

G2O_CORE_API std::string_view to_string(g2o::io::Format format);

/**
 * @brief FileFilter information for Open/Save Dialog
 *
 */
struct G2O_CORE_API FileFilter {
  FileFilter(std::string filter, Format format);
  std::string filter;  ///< filter string
  Format format;       ///< IO format
  bool operator==(const FileFilter& other) const;
};

/**
 * @brief Maps a file extension to a format value
 *
 * @param extension Filename extension, e.g., json, g2o, xml, bin
 * @return std::optional<Format> of the corresponding format, nullopt if cannot
 * match
 */
G2O_CORE_API std::optional<Format> formatForFileExtension(
    std::string_view extension);

/**
 * @brief Get the filters for file open dialogs.
 *
 * @param open true, if opening files, false for save
 * @return std::vector<std::string> Filters
 */
G2O_CORE_API std::vector<FileFilter> getFileFilter(bool open);

}  // namespace g2o::io

#endif
