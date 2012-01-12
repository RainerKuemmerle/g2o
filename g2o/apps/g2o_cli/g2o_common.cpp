// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
#include <dlfcn.h>
static Dl_info info;
#define PATH_SEPARATOR ":"
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
#if defined (UNIX) || defined(CYGWIN)
    if (dladdr(&info, &info) != 0) {
      typesPath = getDirname(info.dli_fname);
    }
#else // Windows
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
  string solversPath;

  if (envSolversPath != NULL) {
      solversPath = envSolversPath;
  } else {
#if defined (UNIX) || defined(CYGWIN)
    if (dladdr(&info, &info) != 0) {
      solversPath = getDirname(info.dli_fname);
    } else {
      solversPath = G2O_DEFAULT_SOLVERS_DIR_;
    }
#else // Windows
    char libFilename[MAX_PATH + 1];
    HMODULE instance = getMyInstance();
    if (instance && GetModuleFileName(instance, libFilename, MAX_PATH) > 0) {
      solversPath = getDirname(libFilename);
    } else {
      solversPath = G2O_DEFAULT_SOLVERS_DIR_;
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
