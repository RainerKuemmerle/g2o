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

#ifndef G2O_STRING_TOOLS_H
#define G2O_STRING_TOOLS_H

#include <algorithm>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

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
 * convert a string into an other type.
 */
template <typename T>
bool convertString(const std::string& s, T& x,
                   bool failIfLeftoverChars = true) {
  std::istringstream i(s);
  char c;
  if (!(i >> x) || (failIfLeftoverChars && i.get(c))) return false;
  return true;
}

/**
 * convert a string into an other type.
 * Return the converted value. Throw error if parsing is wrong.
 */
template <typename T>
T stringToType(const std::string& s, bool failIfLeftoverChars = true) {
  T x;
  convertString(s, x, failIfLeftoverChars);
  return x;
}

/**
 * return true, if str starts with substr
 */
G2O_STUFF_API bool strStartsWith(const std::string& str,
                                 const std::string& substr);

/**
 * return true, if str ends with substr
 */
G2O_STUFF_API bool strEndsWith(const std::string& str,
                               const std::string& substr);

/**
 * expand the given filename like a posix shell, e.g., ~ $CARMEN_HOME and other
 * will get expanded. Also command substitution, e.g. `pwd` will give the
 * current directory.
 */
G2O_STUFF_API std::string strExpandFilename(const std::string& filename);

/**
 * split a string into token based on the characters given in delim
 */
G2O_STUFF_API std::vector<std::string> strSplit(const std::string& s,
                                                const std::string& delim);

/**
 * @brief Join into a string using a delimeter
 *
 * @tparam Iterator
 * @tparam std::iterator_traits<Iterator>::value_type
 * @param b begin of the range for output
 * @param e end of the range for output
 * @param delimiter will be inserted in between elements
 * @return std::string joined string
 */
template <typename Iterator,
          typename Value = typename std::iterator_traits<Iterator>::value_type>
std::string strJoin(Iterator b, Iterator e, const std::string& delimiter = "") {
  if (b == e) return "";
  std::ostringstream os;
  std::copy(b, std::prev(e),
            std::ostream_iterator<Value>(os, delimiter.c_str()));
  b = std::prev(e);
  if (b != e) {
    os << *b;
  }
  return os.str();
}

/**
 * read a line from is into currentLine.
 * @return the number of characters read into currentLine (excluding newline),
 * -1 on eof()
 */
G2O_STUFF_API int readLine(std::istream& is, std::stringstream& currentLine);

/**
 * read from string until the end of a line is reached.
 */
G2O_STUFF_API void skipLine(std::istream& is);

// @}

}  // namespace g2o

#endif
