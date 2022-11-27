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

#include "dl_wrapper.h"

#include <sys/types.h>

#include <algorithm>
#include <cstdio>
#include <iostream>

#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/macros.h"

#if defined(UNIX) || defined(CYGWIN)
#include <dlfcn.h>
#endif

#ifdef __APPLE__
#define SO_EXT "dylib"
#define SO_EXT_LEN 5
#elif defined(WINDOWS) || defined(CYGWIN)
#define SO_EXT "dll"
#define SO_EXT_LEN 3
#else  // Linux
#define SO_EXT "so"
#define SO_EXT_LEN 2
#endif

namespace g2o {

int DlWrapper::openLibraries(const std::string& directory,
                             const std::string& pattern) {
  // cerr << "# loading libraries from " << directory << "\t pattern: " <<
  // pattern << endl;
  const std::string searchPattern =
      pattern.empty() ? directory + "/*" : directory + "/" + pattern;
  const std::vector<std::string> matchingFiles =
      getFilesByPattern(searchPattern.c_str());

  int numLibs = 0;
  for (const auto& filename : matchingFiles) {
    if (find(filenames_.begin(), filenames_.end(), filename) !=
        filenames_.end())
      continue;

      // If we are doing a release build, the wildcards will pick up the
      // suffixes; unfortunately the "_rd" extension means that we
      // don't seem to be able to filter out the incompatible files using a
      // wildcard expansion to wordexp.

#ifndef G2O_LIBRARY_POSTFIX
    if ((filename.rfind(std::string("_d.") + SO_EXT) ==
         filename.length() - 3 - SO_EXT_LEN) ||
        (filename.rfind(std::string("_rd.") + SO_EXT) ==
         filename.length() - 4 - SO_EXT_LEN) ||
        (filename.rfind(std::string("_s.") + SO_EXT) ==
         filename.length() - 3 - SO_EXT_LEN))
      continue;
#endif

    // open the lib
    // cerr << "loading " << filename << endl;
    if (openLibrary(filename)) numLibs++;
  }

  return numLibs;
}

void DlWrapper::clear() {
  for (auto& handle : handles_) {
#if defined(UNIX) || defined(CYGWIN)
    dlclose(handle);
#elif defined(WINDOWS)
    FreeLibrary(handle);
#endif
  }
  filenames_.clear();
  handles_.clear();
}

bool DlWrapper::openLibrary(const std::string& filename) {
#if defined(UNIX) || defined(CYGWIN)
  void* handle = dlopen(filename.c_str(), RTLD_LAZY);
  if (!handle) {
    std::cerr << __PRETTY_FUNCTION__ << " Cannot open library: " << dlerror()
              << '\n';
    return false;
  }
#elif defined(WINDOWS)
  HMODULE handle = LoadLibrary(filename.c_str());
  if (!handle) {
    std::cerr << __PRETTY_FUNCTION__ << " Cannot open library." << std::endl;
    return false;
  }
#endif

  // cerr << "loaded " << filename << endl;

  filenames_.push_back(filename);
  handles_.push_back(handle);
  return true;
}

}  // end namespace g2o
