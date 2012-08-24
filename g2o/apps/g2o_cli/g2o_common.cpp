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

#include "g2o_common.h"

#include "dl_wrapper.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/filesys_tools.h"

#include <vector>
#include <iostream>
#include <cstdlib>
using namespace ::std;

/*
 * setting up the library filename patterns for the different OS
 */
#ifdef __APPLE__
#define SO_EXT "dylib"
#elif defined (WINDOWS) || defined (CYGWIN)
#define SO_EXT "dll"
#else // Linux
#define SO_EXT "so"
#endif

// This is used to determine where this library is
#if defined (UNIX) || defined(CYGWIN)
# if (defined UNIX)
   // dladdr is not available on a recent installation of Cygwin
#  ifndef _GNU_SOURCE
#    define _GNU_SOURCE
#  endif
#  include <dlfcn.h>
   static Dl_info info;
#  endif
# define PATH_SEPARATOR ":"
#else // WINDOWS
#define PATH_SEPARATOR ";"

static void fakeFunctionForWindows() {}

HMODULE getMyInstance()
{
  MEMORY_BASIC_INFORMATION mbi;
  if (VirtualQuery((const void *)&fakeFunctionForWindows, &mbi, sizeof(mbi))) {
    return (HMODULE) mbi.AllocationBase;
  }
  return NULL;
}
#endif

// This can occur if we are doing a release build, and the release
// postfix is empty
#ifndef G2O_LIBRARY_POSTFIX
#define G2O_LIBRARY_POSTFIX ""
#endif

static const string TYPES_PATTERN=string("*_types_*")+string(G2O_LIBRARY_POSTFIX)+string(".")+string(SO_EXT);
static const string SOLVERS_PATTERN=string("*_solver_*")+string(G2O_LIBRARY_POSTFIX)+string(".")+string(SO_EXT);

namespace g2o {

void findArguments(const std::string& option, vector<string>& args, int argc, char** argv)
{
  args.clear();
  for (int i = 0; i < argc; ++i) {
    if (argv[i] == option && i + 1 < argc) {
      args.push_back(argv[i+1]);
    }
  }
}

void loadStandardTypes(DlWrapper& dlTypesWrapper, int argc, char** argv)
{
  char * envTypesPath = getenv("G2O_TYPES_DIR");
  string typesPath;

  if (envTypesPath != NULL) {
    typesPath = envTypesPath;
  } else {
    typesPath = G2O_DEFAULT_TYPES_DIR_;
#if (defined UNIX)
    if (dladdr(&info, &info) != 0) {
      typesPath = getDirname(info.dli_fname);
    }
#elif (defined WINDOWS)
    char libFilename[MAX_PATH + 1];
    HMODULE instance = getMyInstance();
    if (instance && GetModuleFileName(instance, libFilename, MAX_PATH) > 0) {
      typesPath = getDirname(libFilename);
    }
#endif
  }

  vector<string> paths = strSplit(typesPath, PATH_SEPARATOR);
  for (vector<string>::const_iterator it = paths.begin(); it != paths.end(); ++it) {
    if (it->size() > 0)
      dlTypesWrapper.openLibraries(*it, TYPES_PATTERN);
  }

  vector<string> libs;
  if (argc > 0 && argv != 0)
    findArguments("-typeslib", libs, argc, argv);
  for (vector<string>::const_iterator it = libs.begin(); it != libs.end(); ++it) {
    cerr << "Loading types " << *it << endl;
    dlTypesWrapper.openLibrary(*it);
  }
}

void loadStandardSolver(DlWrapper& dlSolverWrapper, int argc, char** argv)
{
  char * envSolversPath = getenv("G2O_SOLVERS_DIR");
  string solversPath = G2O_DEFAULT_SOLVERS_DIR_;

  if (envSolversPath != NULL) {
      solversPath = envSolversPath;
  } else {
#if (defined UNIX)
    if (dladdr(&info, &info) != 0) {
      solversPath = getDirname(info.dli_fname);
    }
#elif (defined WINDOWS)
    char libFilename[MAX_PATH + 1];
    HMODULE instance = getMyInstance();
    if (instance && GetModuleFileName(instance, libFilename, MAX_PATH) > 0) {
      solversPath = getDirname(libFilename);
    }
#endif
  }

  vector<string> paths = strSplit(solversPath, PATH_SEPARATOR);
  for (vector<string>::const_iterator it = paths.begin(); it != paths.end(); ++it) {
    if (it->size() > 0)
      dlSolverWrapper.openLibraries(*it, SOLVERS_PATTERN);
  }

  vector<string> libs;
  if (argc > 0 && argv != 0)
    findArguments("-solverlib", libs, argc, argv);
  for (vector<string>::const_iterator it = libs.begin(); it != libs.end(); ++it) {
    cerr << "Loading solver " << *it << endl;
    dlSolverWrapper.openLibrary(*it);
  }
}

} // end namespace
