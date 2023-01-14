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

#include "command_args.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>

#include "misc.h"
#include "os_specific.h"
#include "string_tools.h"

namespace g2o {

namespace {

template <typename T>
void readVector(const std::string& s, std::vector<T>& v) {
  v.clear();

  const std::vector<std::string> elements = strSplit(s, ",;");
  for (const std::string& s : elements) {
    T val = stringToType<T>(s);
    v.emplace_back(val);
  }
}

template <typename T>
std::string writeVectorAsString(const std::vector<T>& v) {
  std::stringstream os;
  if (!v.empty()) os << v[0];
  for (size_t i = 1; i < v.size(); i++) os << "," << v[i];
  return os.str();
}

template <typename T>
void parseArgument(const std::string& input, CommandArgs::CommandArgument& ca) {
  T aux;
  const bool convertStatus = convertString(input, aux);
  if (convertStatus) {
    T* data = static_cast<T*>(ca.data);
    *data = aux;
  }
}

template <typename T>
void parseVector(const std::string& input, CommandArgs::CommandArgument& ca) {
  std::vector<T> aux;
  readVector(input, aux);
  const bool convertStatus = !aux.empty();
  if (convertStatus) {
    auto data = static_cast<std::vector<T>*>(ca.data);
    *data = aux;
  }
}

template <typename T>
std::string argument2String(const CommandArgs::CommandArgument& ca) {
  T* data = static_cast<T*>(ca.data);
  std::stringstream auxStream;
  auxStream << *data;
  return auxStream.str();
}

}  // namespace

enum CommandArgumentType {
  kCatDouble,
  kCatFloat,
  kCatInt,
  kCatString,
  kCatBool,
  kCatVectorInt,
  kCatVectorDouble
};

bool CommandArgs::parseArgs(int argc, char** argv, bool exitOnError) {
  progName_ = argv[0];
  int i;
  for (i = 1; i < argc; i++) {
    std::string name = argv[i];

    if (name[0] != '-') {  // each param has to start with at least one dash
      // cerr << "Error: expecting parameter, got " << name << endl;
      // printHelp(cerr);
      // if (exitOnError)
      // exit(1);
      // return false;
      break;
    }
    /* first check whether it's -- and we should not continue parsing */
    if (name == "--") {
      ++i;
      break;
    }

    const std::string::size_type dashPos = name.find_first_not_of('-');
    if (dashPos != std::string::npos) name = name.substr(dashPos);

    if (name == "help" || name == "h") {
      printHelp(std::cout);
      exit(0);
    } else {
      // command line argument parsing
      auto it = args_.begin();
      for (; it != args_.end(); ++it) {
        if (it->name == name) {
          if (it->type == kCatBool) {
            if (!it->parsed) {
              bool* data = static_cast<bool*>(it->data);
              *data = !(*data);
            }
            it->parsed = true;
          } else {
            if (i >= argc - 1) {
              std::cerr << "Argument " << name << "needs value.\n";
              printHelp(std::cerr);
              if (exitOnError) exit(1);
              return false;
            }
            i++;
            str2arg(argv[i], *it);
            it->parsed = true;
          }
          break;
        }
      }
      if (it == args_.end()) {
        std::cerr << "Error: Unknown Option '" << name
                  << "' (use -help to get list of options).\n";
        if (exitOnError) exit(1);
        return false;
      }
    }
  }  // for argv[i]

  if (static_cast<int>(leftOvers_.size()) > argc - i) {
    std::cerr << "Error: program requires parameters" << std::endl;
    printHelp(std::cerr);
    if (exitOnError) exit(1);
    return false;
  }
  for (size_t j = 0; (i < argc && j < leftOvers_.size()); i++, j++) {
    auto* s = static_cast<std::string*>(leftOvers_[j].data);
    *s = argv[i];
    leftOvers_[j].parsed = true;
  }

  // the optional leftOvers
  for (size_t j = 0; (i < argc && j < leftOversOptional_.size()); i++, j++) {
    auto* s = static_cast<std::string*>(leftOversOptional_[j].data);
    *s = argv[i];
    leftOversOptional_[j].parsed = true;
  }

  return true;
}

void CommandArgs::param(const std::string& name, bool& p, bool defValue,
                        const std::string& desc) {
  CommandArgument ca;
  ca.name = name;
  ca.description = desc;
  ca.type = kCatBool;
  ca.data = static_cast<void*>(&p);
  ca.parsed = false;
  p = defValue;
  args_.push_back(ca);
}

void CommandArgs::param(const std::string& name, int& p, int defValue,
                        const std::string& desc) {
  CommandArgument ca;
  ca.name = name;
  ca.description = desc;
  ca.type = kCatInt;
  ca.data = static_cast<void*>(&p);
  ca.parsed = false;
  p = defValue;
  args_.push_back(ca);
}

void CommandArgs::param(const std::string& name, float& p, float defValue,
                        const std::string& desc) {
  CommandArgument ca;
  ca.name = name;
  ca.description = desc;
  ca.type = kCatFloat;
  ca.data = static_cast<void*>(&p);
  ca.parsed = false;
  p = defValue;
  args_.push_back(ca);
}

void CommandArgs::param(const std::string& name, double& p, double defValue,
                        const std::string& desc) {
  CommandArgument ca;
  ca.name = name;
  ca.description = desc;
  ca.type = kCatDouble;
  ca.data = static_cast<void*>(&p);
  ca.parsed = false;
  p = defValue;
  args_.push_back(ca);
}

void CommandArgs::param(const std::string& name, std::string& p,
                        const std::string& defValue, const std::string& desc) {
  CommandArgument ca;
  ca.name = name;
  ca.description = desc;
  ca.type = kCatString;
  ca.data = static_cast<void*>(&p);
  ca.parsed = false;
  p = defValue;
  args_.push_back(ca);
}

void CommandArgs::param(const std::string& name, std::vector<int>& p,
                        const std::vector<int>& defValue,
                        const std::string& desc) {
  CommandArgument ca;
  ca.name = name;
  ca.description = desc;
  ca.type = kCatVectorInt;
  ca.data = static_cast<void*>(&p);
  ca.parsed = false;
  p = defValue;
  args_.push_back(ca);
}

void CommandArgs::param(const std::string& name, std::vector<double>& p,
                        const std::vector<double>& defValue,
                        const std::string& desc) {
  CommandArgument ca;
  ca.name = name;
  ca.description = desc;
  ca.type = kCatVectorDouble;
  ca.data = static_cast<void*>(&p);
  ca.parsed = false;
  p = defValue;
  args_.push_back(ca);
}

void CommandArgs::printHelp(std::ostream& os) {
  if (!banner_.empty()) os << banner_ << std::endl;
  os << "Usage: " << progName_ << (args_.empty() ? " " : " [options] ");
  for (size_t i = 0; i < leftOvers_.size(); ++i) {
    if (i > 0) os << " ";
    os << leftOvers_[i].name;
  }
  if (!leftOversOptional_.empty()) {
    if (!leftOvers_.empty()) os << " ";
    for (size_t i = 0; i < leftOversOptional_.size(); ++i) {
      if (i > 0) os << " ";
      os << "[" << leftOversOptional_[i].name << "]";
    }
  }
  os << std::endl << std::endl;
  os << "General options:" << std::endl;
  os << "-------------------------------------------" << std::endl;
  os << "-help / -h           Displays this help." << std::endl << std::endl;
  if (!args_.empty()) {
    os << "Program Options:" << std::endl;
    os << "-------------------------------------------" << std::endl;
    // build up option string to print as table
    std::vector<std::pair<std::string, std::string> > tableStrings;
    tableStrings.reserve(args_.size());
    size_t maxArgLen = 0;
    for (const auto& arg : args_) {
      if (arg.type != kCatBool) {
        const std::string defaultValueStr = arg2str(arg);
        if (!defaultValueStr.empty())
          tableStrings.emplace_back(
              arg.name + " " + type2str(arg.type),
              arg.description + " (default: " + defaultValueStr + ")");
        else
          tableStrings.emplace_back(arg.name + " " + type2str(arg.type),
                                    arg.description);
      } else
        tableStrings.emplace_back(arg.name, arg.description);
      maxArgLen = (std::max)(maxArgLen, tableStrings.back().first.size());
    }
    sort(tableStrings.begin(), tableStrings.end(),
         CmpPairFirst<std::string, std::string>());
    maxArgLen += 3;
    for (const auto& tableString : tableStrings) {
      os << "-" << tableString.first;
      for (size_t l = tableString.first.size(); l < maxArgLen; ++l) os << " ";
      os << tableString.second << std::endl;
    }
    // TODO(goki): should output description for leftOver params
  }
}

void CommandArgs::setBanner(const std::string& banner) { banner_ = banner; }

void CommandArgs::paramLeftOver(const std::string& name, std::string& p,
                                const std::string& defValue,
                                const std::string& desc, bool optional) {
  CommandArgument ca;
  ca.name = name;
  ca.description = desc;
  ca.type = kCatString;
  ca.data = static_cast<void*>(&p);
  ca.parsed = false;
  ca.optional = optional;
  p = defValue;
  if (optional)
    leftOversOptional_.push_back(ca);
  else
    leftOvers_.push_back(ca);
}

const char* CommandArgs::type2str(int t) {
  switch (t) {
    case kCatDouble:
      return "<double>";
    case kCatFloat:
      return "<float>";
    case kCatInt:
      return "<int>";
    case kCatString:
      return "<string>";
    case kCatBool:
      return "<bool>";
    case kCatVectorInt:
      return "<vector_int>";
    case kCatVectorDouble:
      return "<vector_double>";
  }
  return "";
}

void CommandArgs::str2arg(const std::string& input, CommandArgument& ca) {
  switch (ca.type) {
    case kCatFloat:
      parseArgument<float>(input, ca);
      break;
    case kCatDouble:
      parseArgument<double>(input, ca);
      break;
    case kCatInt:
      parseArgument<int>(input, ca);
      break;
    case kCatBool:
      parseArgument<bool>(input, ca);
      break;
    case kCatString: {
      auto* data = static_cast<std::string*>(ca.data);
      *data = input;
    } break;
    case kCatVectorInt: {
      parseVector<int>(input, ca);
    } break;
    case kCatVectorDouble: {
      parseVector<double>(input, ca);
    } break;
  }
}

std::string CommandArgs::arg2str(const CommandArgument& ca) {
  switch (ca.type) {
    case kCatFloat:
      return argument2String<float>(ca);
    case kCatDouble:
      return argument2String<double>(ca);
    case kCatInt:
      return argument2String<int>(ca);
    case kCatBool:
      return argument2String<bool>(ca);
    case kCatString: {
      auto* data = static_cast<std::string*>(ca.data);
      return *data;
    }
    case kCatVectorInt: {
      auto* data = static_cast<std::vector<int>*>(ca.data);
      return writeVectorAsString(*data);
    }
    case kCatVectorDouble: {
      auto* data = static_cast<std::vector<double>*>(ca.data);
      return writeVectorAsString(*data);
    }
  }
  return "";
}

bool CommandArgs::parsedParam(const std::string& param) const {
  for (const auto& arg : args_)
    if (arg.name == param) return arg.parsed;
  return false;
}

}  // end namespace g2o
