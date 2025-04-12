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

#ifndef G2O_FILESYS_TOOLS_H
#define G2O_FILESYS_TOOLS_H

#include "g2o_stuff_api.h"

/** @addtogroup utils **/
// @{

/** \file filesysTools.h
 * \brief utility functions for handling files, directory on Linux/Unix
 */

#include <regex>
#include <string>
#include <string_view>
#include <vector>

namespace g2o {

/**
 * get filename extension (the part after the last .), e.g.
 * the extension of file.txt is txt
 */
G2O_STUFF_API std::string getFileExtension(std::string_view filename);

/**
 * get the filename without the extension.
 * file.txt -> file
 */
G2O_STUFF_API std::string getPureFilename(std::string_view filename);

/**
 * change the fileextension of a given filename.
 * Only if filename contains an extension, otherwise filename is returned.
 */
G2O_STUFF_API std::string changeFileExtension(std::string_view filename,
                                              std::string_view newExt);

/**
 * return the basename of the given filename
 * /etc/fstab -> fstab
 */
G2O_STUFF_API std::string getBasename(std::string_view filename);

/**
 * return the directory of a given filename
 * /etc/fstab -> /etc
 */
G2O_STUFF_API std::string getDirname(std::string_view filename);

/**
 * check if file exists (note a directory is also a file)
 * @param regular to check for regular file instead
 */
G2O_STUFF_API bool fileExists(std::string_view filename, bool regular = false);

/**
 * return all files that match a given regexp in the directory.
 */
G2O_STUFF_API std::vector<std::string> getFilesByPattern(
    std::string_view directory, const std::regex& pattern);

}  // namespace g2o
// @}
#endif
