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

#ifndef G2O_STRING_TOOLS_H
#define G2O_STRING_TOOLS_H

#include <string>
#include <sstream>
#include <cstdlib>
#include <vector>

#include "macros.h"
#include "g2o_stuff_api.h"

namespace g2o {

/** @addtogroup utils **/
// @{

/** \file stringTools.h
 * \brief utility functions for handling strings
 */

/**
 * remove whitespaces from the start/end of a string
 */
G2O_STUFF_API std::string trim(const std::string& s);

/**
 * remove whitespaces from the left side of the string
 */
G2O_STUFF_API std::string trimLeft(const std::string& s);

/**
 * remove whitespaced from the right side of the string
 */
G2O_STUFF_API std::string trimRight(const std::string& s);

/**
 * convert the string to lower case
 */
G2O_STUFF_API std::string strToLower(const std::string& s);

/**
 * convert a string to upper case
 */
G2O_STUFF_API std::string strToUpper(const std::string& s);

/**
 * read integer values (seperated by spaces) from a string and store
 * them in the given OutputIterator.
 */
template <typename OutputIterator>
OutputIterator readInts(const char* str, OutputIterator out)
{
  char* cl  = (char*)str;
  char* cle = cl;
  while (1) {
    int id = strtol(cl, &cle, 10);
    if (cl == cle)
      break;
    *out++ = id;
    cl = cle;
  }
  return out;
}

/**
 * read float values (seperated by spaces) from a string and store
 * them in the given OutputIterator.
 */
template <typename OutputIterator>
OutputIterator readFloats(const char* str, OutputIterator out)
{
  char* cl  = (char*)str;
  char* cle = cl;
  while (1) {
    double val = strtod(cl, &cle);
    if (cl == cle)
      break;
    *out++ = val;
    cl = cle;
  }
  return out;
}

/**
 * format a string and return a std::string.
 * Format is just like printf, see man 3 printf
 */
std::string formatString(const char* fmt, ...) G2O_ATTRIBUTE_FORMAT12;

/**
 * replacement function for sprintf which fills a std::string instead of a char*
 */
int strPrintf(std::string& str, const char* fmt, ...) G2O_ATTRIBUTE_FORMAT23;

/**
 * convert a string into an other type.
 */
template<typename T>
bool convertString(const std::string& s, T& x, bool failIfLeftoverChars = true)
{
  std::istringstream i(s);
  char c;
  if (!(i >> x) || (failIfLeftoverChars && i.get(c)))
    return false;
  return true;
}

/**
 * convert a string into an other type.
 * Return the converted value. Throw error if parsing is wrong.
 */
template<typename T>
T stringToType(const std::string& s, bool failIfLeftoverChars = true)
{
  T x;
  convertString(s, x, failIfLeftoverChars);
  return x;
}

/**
 * return true, if str starts with substr
 */
bool strStartsWith(const std::string & str, const std::string& substr);

/**
 * return true, if str ends with substr
 */
bool strEndsWith(const std::string & str, const std::string& substr);

/**
 * expand the given filename like a posix shell, e.g., ~ $CARMEN_HOME and other will get expanded.
 * Also command substitution, e.g. `pwd` will give the current directory.
 */
std::string strExpandFilename(const std::string& filename);

/**
 * split a string into token based on the characters given in delim
 */
G2O_STUFF_API std::vector<std::string> strSplit(const std::string& s, const std::string& delim);

/**
 * read a line from is into currentLine.
 * @return the number of characters read into currentLine (excluding newline), -1 on eof()
 */
G2O_STUFF_API int readLine(std::istream& is, std::stringstream& currentLine);

// @}

} // end namespace

#endif
