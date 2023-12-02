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

#include <cstdio>
#include <regex>

#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/logger.h"

#if defined(UNIX) || defined(CYGWIN)
#include <dlfcn.h>
#endif

namespace g2o {

DlWrapper::DlWrapper() {}

DlWrapper::~DlWrapper() {
  // clear();
}

int DlWrapper::openLibraries(const std::string& directory,
                             const std::string& pattern) {
  G2O_TRACE("Loading libraries from {} pattern {}", directory, pattern);
  std::vector<std::string> matchingFiles =
      getFilesByPattern(directory, std::regex(pattern));

  int numLibs = 0;
  for (const std::string& filename : matchingFiles) {
    if (_filenames.count(filename) != 0) continue;

    // open the lib
    G2O_TRACE("Loading {}", filename);
    if (openLibrary(filename)) numLibs++;
  }

  return numLibs;
}

void DlWrapper::clear() {
#if defined(UNIX) || defined(CYGWIN)
  for (size_t i = 0; i < _handles.size(); ++i) {
    dlclose(_handles[i]);
  }
#elif defined(WINDOWS)
  for (size_t i = 0; i < _handles.size(); ++i) {
    FreeLibrary(_handles[i]);
  }
#endif
  _filenames.clear();
  _handles.clear();
}

bool DlWrapper::openLibrary(const std::string& filename) {
#if defined(UNIX) || defined(CYGWIN)
  void* handle = dlopen(filename.c_str(), RTLD_LAZY);
  if (!handle) {
    G2O_ERROR("Cannot open library: {} Error: {}", filename, dlerror());
    return false;
  }
#elif defined(WINDOWS)
  HMODULE handle = LoadLibrary(filename.c_str());
  if (!handle) {
    G2O_ERROR("Cannot open library: {}", filename);
    return false;
  }
#else
#warning "No implementation for openLibrary found"
  return false;
#endif

  // cerr << "loaded " << filename << endl;

  _filenames.insert(filename);
  _handles.push_back(handle);
  return true;
}

}  // end namespace g2o
