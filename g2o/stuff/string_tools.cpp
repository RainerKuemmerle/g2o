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

#include "string_tools.h"
#include "os_specific.h"
#include "macros.h"

#include <cctype>
#include <string>
#include <cstdarg>
#include <cstring>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <iterator>

#ifdef UNIX
#include <wordexp.h>
#endif

namespace g2o {

using namespace std;

std::string trim(const std::string& s)
{
  if(s.length() == 0)
    return s;
  string::size_type b = s.find_first_not_of(" \t\n");
  string::size_type e = s.find_last_not_of(" \t\n");
  if(b == string::npos)
    return "";
  return std::string(s, b, e - b + 1);
}

std::string trimLeft(const std::string& s)
{
  if(s.length() == 0)
    return s;
  string::size_type b = s.find_first_not_of(" \t\n");
  string::size_type e = s.length() - 1;
  if(b == string::npos)
    return "";
  return std::string(s, b, e - b + 1);
}

std::string trimRight(const std::string& s)
{
  if(s.length() == 0)
    return s;
  string::size_type b = 0;
  string::size_type e = s.find_last_not_of(" \t\n");
  if(b == string::npos)
    return "";
  return std::string(s, b, e - b + 1);
}

std::string strToLower(const std::string& s)
{
  string ret;
  std::transform(s.begin(), s.end(), back_inserter(ret), (int(*)(int)) std::tolower);
  return ret;
}

std::string strToUpper(const std::string& s)
{
  string ret;
  std::transform(s.begin(), s.end(), back_inserter(ret), (int(*)(int)) std::toupper);
  return ret;
}

std::string formatString(const char* fmt, ...)
{
  char* auxPtr = NULL;
  va_list arg_list;
  va_start(arg_list, fmt);
  int numChar = vasprintf(&auxPtr, fmt, arg_list);
  va_end(arg_list);
  string retString;
  if (numChar != -1)
    retString = auxPtr;
  else {
    cerr << __PRETTY_FUNCTION__ << ": Error while allocating memory" << endl;
  }
  free(auxPtr);
  return retString;
}

int strPrintf(std::string& str, const char* fmt, ...)
{
  char* auxPtr = NULL;
  va_list arg_list;
  va_start(arg_list, fmt);
  int numChars = vasprintf(&auxPtr, fmt, arg_list);
  va_end(arg_list);
  str = auxPtr;
  free(auxPtr);
  return numChars;
}

std::string strExpandFilename(const std::string& filename)
{

  #ifdef UNIX
  string result = filename;
  wordexp_t p;

  wordexp(filename.c_str(), &p, 0);
  if(p.we_wordc > 0) {
    result = p.we_wordv[0];
  }
  wordfree(&p);
  return result;
  #endif

  #ifdef WINDOWS
  (void) filename;
  std::cerr << "WARNING: " << __PRETTY_FUNCTION__ << " not implemented" << std::endl;
  return std::string();
  #endif

}

std::vector<std::string> strSplit(const std::string& str, const std::string& delimiters)
{
  std::vector<std::string> tokens;
  string::size_type lastPos = 0;
  string::size_type pos     = 0;

  do {
    pos = str.find_first_of(delimiters, lastPos);
    tokens.push_back(str.substr(lastPos, pos - lastPos));
    lastPos = pos + 1;
  }  while (string::npos != pos);

  return tokens;
}

bool strStartsWith(const std::string& s, const std::string& start)
{
  if (s.size() < start.size())
    return false;
  return equal(start.begin(), start.end(), s.begin());
}

bool strEndsWith(const std::string& s, const std::string& end)
{
  if (s.size() < end.size())
    return false;
  return equal(end.rbegin(), end.rend(), s.rbegin());
}

int readLine(std::istream& is, std::stringstream& currentLine)
{
  if (is.eof())
    return -1;
  currentLine.str("");
  currentLine.clear();
  is.get(*currentLine.rdbuf());
  if (is.fail()) // fail is set on empty lines
    is.clear();
  G2O_FSKIP_LINE(is); // read \n not read by get()
  return currentLine.str().size();
}

} // end namespace
