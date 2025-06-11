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

#include "string_tools.h"

#include <algorithm>
#include <cctype>
#include <iostream>
#include <iterator>
#include <string>

#ifdef __APPLE__
#include <TargetConditionals.h>
#endif

#if (defined(UNIX) || defined(CYGWIN)) && \
    !(defined(ANDROID) || TARGET_IPHONE_SIMULATOR || TARGET_OS_IPHONE)
#define HAS_WORDEXP
#include <wordexp.h>
#else
#include "g2o/stuff/logger.h"
#endif

namespace g2o {

using namespace std;

std::string trim(const std::string& s) {
  if (s.length() == 0) return s;
  string::size_type b = s.find_first_not_of(" \t\n");
  string::size_type e = s.find_last_not_of(" \t\n");
  if (b == string::npos) return "";
  return std::string(s, b, e - b + 1);
}

std::string trimLeft(const std::string& s) {
  if (s.length() == 0) return s;
  string::size_type b = s.find_first_not_of(" \t\n");
  string::size_type e = s.length() - 1;
  if (b == string::npos) return "";
  return std::string(s, b, e - b + 1);
}

std::string trimRight(const std::string& s) {
  if (s.length() == 0) return s;
  string::size_type b = 0;
  string::size_type e = s.find_last_not_of(" \t\n");
  if (e == string::npos) return "";
  return std::string(s, b, e - b + 1);
}

std::string strToLower(const std::string& s) {
  string ret;
  ret.reserve(s.size());
  std::transform(s.begin(), s.end(), back_inserter(ret),
                 [](unsigned char c) { return std::tolower(c); });
  return ret;
}

std::string strToUpper(const std::string& s) {
  string ret;
  ret.reserve(s.size());
  std::transform(s.begin(), s.end(), back_inserter(ret),
                 [](unsigned char c) { return std::toupper(c); });
  return ret;
}

std::string strExpandFilename(const std::string& filename) {
#ifdef HAS_WORDEXP
  string result = filename;
  wordexp_t p;

  wordexp(filename.c_str(), &p, 0);
  if (p.we_wordc > 0) {
    result = p.we_wordv[0];
  }
  wordfree(&p);
  return result;
#else
  (void)filename;
  G2O_WARN("not implemented");
  return std::string();
#endif
}

std::vector<std::string> strSplit(const std::string& str,
                                  const std::string& delimiters) {
  std::vector<std::string> tokens;
  if (str.empty()) return tokens;
  string::size_type lastPos = 0;
  string::size_type pos = 0;

  do {
    pos = str.find_first_of(delimiters, lastPos);
    tokens.push_back(str.substr(lastPos, pos - lastPos));
    lastPos = pos + 1;
  } while (string::npos != pos);

  return tokens;
}

bool strStartsWith(const std::string& s, const std::string& start) {
  if (s.size() < start.size()) return false;
  return equal(start.begin(), start.end(), s.begin());
}

bool strEndsWith(const std::string& s, const std::string& end) {
  if (s.size() < end.size()) return false;
  return equal(end.rbegin(), end.rend(), s.rbegin());
}

int readLine(std::istream& is, std::stringstream& currentLine) {
  if (is.eof()) return -1;
  currentLine.str("");
  currentLine.clear();
  is.get(*currentLine.rdbuf());
  if (is.fail())  // fail is set on empty lines
    is.clear();
  skipLine(is);  // read \n not read by get()
  if (currentLine.str().empty() && is.eof()) {
    return -1;
  }
  return static_cast<int>(currentLine.str().size());
}

void skipLine(std::istream& is) {
  char c = ' ';
  while (c != '\n' && is.good() && !is.eof()) is.get(c);
}

}  // namespace g2o
