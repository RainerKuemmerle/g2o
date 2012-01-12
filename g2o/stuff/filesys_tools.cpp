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
#include <Windows.h>
#include <WinBase.h>
#endif

#if defined (UNIX) || defined(CYGWIN)
#include <wordexp.h>
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
  
#elif defined (UNIX) || defined (CYGWIN)

  wordexp_t p;
  wordexp(pattern, &p, 0);

  // For some reason, wordexp sometimes fails on an APPLE machine to
  // return anything; therefore, run it several times until we do find
  // something - or give up
#ifdef __APPLE__
  for (int k = 0; (k < 5) && (p.we_wordc == 0); k++) {
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
