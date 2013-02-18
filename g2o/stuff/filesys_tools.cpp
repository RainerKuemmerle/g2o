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

/***************************************************************************
 *            filesysTools.cpp
 *
 *  Fr 02 Mär 2007 23:14:08 CET
 *  Copyright 2007 Rainer Kümmerle
 *  Email rk@raikue.net
 ****************************************************************************/
#include "filesys_tools.h"

#include <sys/stat.h>
#include <ctime>
#include <sys/types.h>
#include <cstdio>
#include <iostream>

#ifdef WINDOWS
#include <windows.h>
#include <winbase.h>
#endif

#if (defined (UNIX) || defined(CYGWIN)) && !defined(ANDROID)
#include <wordexp.h>
#endif

#ifdef __APPLE__
//#include <chrono>
//#include <thread>
#endif

using namespace ::std;

namespace g2o {

std::string getFileExtension(const std::string& filename)
{
  std::string::size_type lastDot = filename.find_last_of('.');
  if (lastDot != std::string::npos)
    return filename.substr(lastDot + 1);
  else
    return "";
}

std::string getPureFilename(const std::string& filename)
{
  std::string::size_type lastDot = filename.find_last_of('.');
  if (lastDot != std::string::npos)
    return filename.substr(0, lastDot);
  else
    return filename;
}

std::string getBasename(const std::string& filename)
{
#ifdef WINDOWS
  std::string::size_type lastSlash = filename.find_last_of('\\');
#else
  std::string::size_type lastSlash = filename.find_last_of('/');
#endif
  if (lastSlash != std::string::npos)
    return filename.substr(lastSlash + 1);
  else
    return filename;
}

std::string getDirname(const std::string& filename)
{
#ifdef WINDOWS
  std::string::size_type lastSlash = filename.find_last_of('\\');
#else
  std::string::size_type lastSlash = filename.find_last_of('/');
#endif
  if (lastSlash != std::string::npos)
    return filename.substr(0, lastSlash);
  else
    return "";
}

std::string changeFileExtension(const std::string& filename, const std::string& newExt, bool stripDot)
{
  std::string::size_type lastDot = filename.find_last_of('.');
  if (lastDot != std::string::npos) {
    if (stripDot)
      return filename.substr(0, lastDot) + newExt;
    else
      return filename.substr(0, lastDot + 1) + newExt;
  } else
    return filename;
}

bool fileExists(const char* filename)
{
  struct stat statInfo;
  return (stat(filename, &statInfo) == 0);
}

std::vector<std::string> getFilesByPattern(const char* pattern)
{
  std::vector<std::string> result;

#ifdef WINDOWS

  HANDLE hFind;
  WIN32_FIND_DATA FData;
  if ((hFind = FindFirstFile(pattern, &FData)) != INVALID_HANDLE_VALUE) {
    do {
      result.push_back(FData.cFileName);
    } while (FindNextFile(hFind, &FData));
    FindClose(hFind);
  }
  
#elif (defined (UNIX) || defined (CYGWIN)) && !defined(ANDROID)

  wordexp_t p;
  wordexp(pattern, &p, 0);

  // For some reason, wordexp sometimes fails on an APPLE machine to
  // return anything; therefore, run it several times until we do find
  // something - or give up
#ifdef __APPLE__
  for (int k = 0; (k < 100) && (p.we_wordc == 0); k++) {
    //chrono::milliseconds duration(20);
    //this_thread::sleep_for(duration);
    wordexp(pattern, &p, WRDE_APPEND);
  }
#endif

  result.reserve(p.we_wordc);
  for (size_t i = 0; i < p.we_wordc; ++i)
    result.push_back(p.we_wordv[i]);
  
  wordfree(&p);

#endif

  return result;
}

}
