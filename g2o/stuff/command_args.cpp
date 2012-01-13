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

// File: commandArgs.cpp
// Copyright (c) 2009 Rainer Kümmerle <rk@raikue.net>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Library General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "command_args.h"

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <algorithm>
#include <functional>

#include "os_specific.h"
using namespace std;

namespace g2o {

/** Helper class to sort pair based on first elem */
template<class T1, class T2, class Pred = std::less<T1> >
struct CmpPairFirst {
  bool operator()(const std::pair<T1,T2>& left, const std::pair<T1,T2>& right) {
    return Pred()(left.first, right.first);
  }
};

enum CommandArgumentType
{
  CAT_DOUBLE, CAT_FLOAT, CAT_INT, CAT_STRING, CAT_BOOL, CAT_VECTOR_INT
};

CommandArgs::CommandArgs()
{
}

CommandArgs::~CommandArgs()
{
}

bool CommandArgs::parseArgs(int argc, char** argv, bool exitOnError)
{
  _progName = argv[0];
  int i;
  for (i = 1; i < argc; i++) {
    string name = argv[i];

    if (name[0] != '-') { // each param has to start with at least one dash
      //cerr << "Error: expecting parameter, got " << name << endl;
      //printHelp(cerr);
      //if (exitOnError)
        //exit(1);
      //return false;
      break;
    }
    /* first check whether it's -- and we should not continue parsing */
    if (name == "--") {
      ++i;
      break;
    }

    string::size_type dashPos = name.find_first_not_of('-');
    if (dashPos != string::npos)
      name = name.substr(dashPos);

    if (name == "help" || name == "h") {
      printHelp(cout);
      exit(0);
    }
    else {
      // command line argument parsing
      std::vector<CommandArgument>::iterator it = _args.begin();
      for ( ; it != _args.end(); ++it) {
        if (it->name == name) {
          if (it->type == CAT_BOOL) {
            if (!it->parsed) {
              bool* data = static_cast<bool*>(it->data);
              *data = !(*data);
            }
            it->parsed = true;
          } else {
            if(i >= argc-1) {
              fprintf(stderr, "Argument %s needs value.\n", name.c_str());
              printHelp(cerr);
              if (exitOnError)
                exit(1);
              return false;
            }
            i++;
            str2arg(argv[i], *it);
            it->parsed = true;
          }
          break;
        }
      }
      if (it == _args.end()) {
        fprintf(stderr, "Error: Unknown Option '%s' (use -help to get list of options).\n", name.c_str());
        if (exitOnError)
          exit(1);
        return false;
      }

    }

  } // for argv[i]

  if ((int)_leftOvers.size() > argc - i) {
    cerr << "Error: program requires parameters" << endl;
    printHelp(cerr);
    if (exitOnError)
      exit(1);
    return false;
  }
  for (size_t j = 0; (i < argc && j < _leftOvers.size()); i++, j++) {
    string* s = static_cast<string*>(_leftOvers[j].data);
    *s = argv[i];
  }

  // the optional leftOvers
  for (size_t j = 0; (i < argc && j < _leftOversOptional.size()); i++, j++) {
    string* s = static_cast<string*>(_leftOversOptional[j].data);
    *s  = argv[i];
  }

  return true;
}

void CommandArgs::param(const std::string& name, bool& p, bool defValue, const std::string& desc)
{
  CommandArgument ca;
  ca.name = name;
  ca.description = desc;
  ca.type = CAT_BOOL;
  ca.data = static_cast<void*>(&p);
  ca.parsed = false;
  p = defValue;
  _args.push_back(ca);
}

void CommandArgs::param(const std::string& name, int& p, int defValue, const std::string& desc)
{
  CommandArgument ca;
  ca.name = name;
  ca.description = desc;
  ca.type = CAT_INT;
  ca.data = static_cast<void*>(&p);
  ca.parsed = false;
  p = defValue;
  _args.push_back(ca);
}

void CommandArgs::param(const std::string& name, float& p, float defValue, const std::string& desc)
{
  CommandArgument ca;
  ca.name = name;
  ca.description = desc;
  ca.type = CAT_FLOAT;
  ca.data = static_cast<void*>(&p);
  ca.parsed = false;
  p = defValue;
  _args.push_back(ca);
}

void CommandArgs::param(const std::string& name, double& p, double defValue, const std::string& desc)
{
  CommandArgument ca;
  ca.name = name;
  ca.description = desc;
  ca.type = CAT_DOUBLE;
  ca.data = static_cast<void*>(&p);
  ca.parsed = false;
  p = defValue;
  _args.push_back(ca);
}

void CommandArgs::param(const std::string& name, std::string& p, const std::string& defValue, const std::string& desc)
{
  CommandArgument ca;
  ca.name = name;
  ca.description = desc;
  ca.type = CAT_STRING;
  ca.data = static_cast<void*>(&p);
  ca.parsed = false;
  p = defValue;
  _args.push_back(ca);
}

 void CommandArgs::param(const std::string& name, std::vector<int>& p, const std::vector<int>& defValue, const std::string& desc){
  CommandArgument ca;
  ca.name = name;
  ca.description = desc;
  ca.type = CAT_VECTOR_INT;
  ca.data = static_cast<void*>(&p);
  ca.parsed = false;
  p = defValue;
  _args.push_back(ca);
}

void CommandArgs::printHelp(std::ostream& os)
{
  if (_banner.size())
    os << _banner << endl;
  os << "Usage: " << _progName << (_args.size()>0?" [options] ":" ");
  if (_leftOvers.size() > 0) {
    for (size_t i = 0; i < _leftOvers.size(); ++i) {
      if (i > 0)
        os << " ";
      os << _leftOvers[i].name;
    }
  }
  if (_leftOversOptional.size() > 0) {
    if (_leftOvers.size() > 0)
      os << " ";
    for (size_t i = 0; i < _leftOversOptional.size(); ++i) {
      if (i > 0)
        os << " ";
      os << "[" << _leftOversOptional[i].name << "]";
    }
  }
  os << endl << endl;
  os << "General options:" << endl;
  os << "-------------------------------------------" << endl;
  os << "-help / -h           Displays this help." << endl << endl;
  if (_args.size() > 0) {
    os << "Program Options:" << endl;
    os << "-------------------------------------------" << endl;
    // build up option string to print as table
    vector<pair<string, string> > tableStrings;
    tableStrings.reserve(_args.size());
    size_t maxArgLen = 0;
    for (size_t i = 0; i < _args.size(); ++i) {
      if (_args[i].type != CAT_BOOL)
        tableStrings.push_back(make_pair(_args[i].name + " " + type2str(_args[i].type), _args[i].description));
      else
        tableStrings.push_back(make_pair(_args[i].name, _args[i].description));
      maxArgLen = (std::max)(maxArgLen, tableStrings.back().first.size());
    }
    sort(tableStrings.begin(), tableStrings.end(), CmpPairFirst<string,string>());
    maxArgLen += 3;
    for (size_t i = 0; i < tableStrings.size(); ++i) {
      os << "-" << tableStrings[i].first;
      for (size_t l = tableStrings[i].first.size(); l < maxArgLen; ++l)
        os << " ";
      os << tableStrings[i].second << endl;
    }
    // TODO should output description for leftOver params?
  }
}


void CommandArgs::setBanner(const std::string& banner)
{
  _banner = banner;
}

void CommandArgs::paramLeftOver(const std::string& name, std::string& p, const std::string& defValue, const std::string& desc, bool optional)
{
  CommandArgument ca;
  ca.name = name;
  ca.description = desc;
  ca.type = CAT_STRING;
  ca.data = static_cast<void*>(&p);
  ca.parsed = false;
  ca.optional = optional;
  p = defValue;
  if (optional)
    _leftOversOptional.push_back(ca);
  else
    _leftOvers.push_back(ca);
}

const char* CommandArgs::type2str(int t) const
{
  switch (t) {
    case CAT_DOUBLE:
      return "<double>";
    case CAT_FLOAT:
      return "<float>";
    case CAT_INT:
      return "<int>";
    case CAT_STRING:
      return "<string>";
    case CAT_BOOL:
      return "<bool>";
    case CAT_VECTOR_INT:
      return "<vector_int>";
  }
  return "";
}

void CommandArgs::str2arg(const std::string& input, CommandArgument& ca) const
{
  switch (ca.type) {
    case CAT_FLOAT:
      {
        float aux;
        bool convertStatus = convertString(input, aux);
        if (convertStatus) {
          float* data = static_cast<float*>(ca.data);
          *data = aux;
        }
      }
      break;
    case CAT_DOUBLE:
      {
        double aux;
        bool convertStatus = convertString(input, aux);
        if (convertStatus) {
          double* data = static_cast<double*>(ca.data);
          *data = aux;
        }
      }
      break;
    case CAT_INT:
      {
        int aux;
        bool convertStatus = convertString(input, aux);
        if (convertStatus) {
          int* data = static_cast<int*>(ca.data);
          *data = aux;
        }
      }
      break;
    case CAT_BOOL:
      {
        bool aux;
        bool convertStatus = convertString(input, aux);
        if (convertStatus) {
          bool* data = static_cast<bool*>(ca.data);
          *data = aux;
        }
      }
      break;
    case CAT_STRING:
      {
        string* data = static_cast<string*>(ca.data);
        *data = input;
      }
      break;
    case CAT_VECTOR_INT:
      {
  std::vector<int> aux;
        bool convertStatus = convertString(input, aux);
        if (convertStatus) {
          std::vector<int>* data = static_cast< std::vector<int>* >(ca.data);
          *data = aux;
        }
      }
      break;
  }
}

std::string CommandArgs::arg2str(const CommandArgument& ca) const
{
  switch (ca.type) {
    case CAT_FLOAT:
      {
        float* data = static_cast<float*>(ca.data);
        stringstream auxStream;
        auxStream << *data;
        return auxStream.str();
      }
      break;
    case CAT_DOUBLE:
      {
        double* data = static_cast<double*>(ca.data);
        stringstream auxStream;
        auxStream << *data;
        return auxStream.str();
      }
      break;
    case CAT_INT:
      {
        int* data = static_cast<int*>(ca.data);
        stringstream auxStream;
        auxStream << *data;
        return auxStream.str();
      }
      break;
    case CAT_BOOL:
      {
        bool* data = static_cast<bool*>(ca.data);
        stringstream auxStream;
        auxStream << *data;
        return auxStream.str();
      }
      break;
    case CAT_VECTOR_INT:
      {
  std::vector<int> * data = static_cast< std::vector<int> * >(ca.data);
  stringstream auxStream;
        auxStream << (*data);
        return auxStream.str();
      }
      break;
  }
  return "";
}

std::string CommandArgs::trim(const std::string& s) const
{
  if(s.length() == 0)
    return s;
  string::size_type b = s.find_first_not_of(" \t\n");
  string::size_type e = s.find_last_not_of(" \t\n");
  if(b == string::npos)
    return "";
  return std::string(s, b, e - b + 1);
}

std::istream& operator >> (std::istream& is, std::vector<int>& v){
  string s;
  if (! (is >> s) )
    return is;

  char* cbase=new char[s.length()];
  char* c=cbase;
  char* caux=cbase;

  strcpy(c, s.c_str());
  v.clear();
  bool hasNextValue=true;
  while(hasNextValue){
    int i=strtol(c,&caux,10);
    if (c!=caux){
      c=caux;
      c++;
      v.push_back(i);
    } else
      hasNextValue = false;
  }
  delete [] cbase;
  return is;
}

std::ostream& operator << (std::ostream& os, const std::vector<int>& v){
  if (v.size()){
    os << v[0];
  }
  for (size_t i=1; i<v.size(); i++){
    os << "," << v[i];
  }
  return os;
}

}
