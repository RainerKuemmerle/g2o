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

// File: commandArgs.h
// Copyright (c) 2009 Rainer Kümmerle <rk@raikue.net>
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef G2O_COMMAND_ARGS_H
#define G2O_COMMAND_ARGS_H

#include <string>
#include <vector>
#include <iostream>
#include <sstream>

#include "g2o_stuff_api.h"

namespace g2o {

/**
 * \brief Command line parsing of argc and argv.
 *
 * Parse the command line to get the program options. Additionally,
 * we can store the config in a file and reload a parameter set from
 * this file.
 */
class G2O_STUFF_API CommandArgs
{
  public:
    struct CommandArgument
    {
      std::string name;
      std::string description;
      int type;
      void* data;
      bool parsed;
      bool optional;
      CommandArgument() : name(""), description(""), type(0), data(0), parsed(false), optional(false)
      {}
    };
  public:
    CommandArgs();
    virtual ~CommandArgs();

    /**
     * parse the command line for the requested parameters.
     * @param argc the number of params
     * @param argv the value array
     * @param exitOnError call exit() if the parsing fails
     * @return true, if parsing was correct
     */
    bool parseArgs(int argc, char** argv, bool exitOnError = true);

    /** add a bool parameter, if found on the command line, will toggle defValue */
    void param(const std::string& name, bool& p, bool defValue, const std::string& desc);
    /** add a int parameter */
    void param(const std::string& name, int& p, int defValue, const std::string& desc);
    /** add a float parameter */
    void param(const std::string& name, float& p, float defValue, const std::string& desc);
    /** add a float parameter */
    void param(const std::string& name, double& p, double defValue, const std::string& desc);
    /** add a string parameter */
    void param(const std::string& name, std::string& p, const std::string& defValue, const std::string& desc);
    /** add an int vector parameter */
    void param(const std::string& name, std::vector<int>& p, const std::vector<int>& defValue, const std::string& desc);
    /** add a param wich is specified as a plain argument */
    void paramLeftOver(const std::string& name, std::string& p, const std::string& defValue, const std::string& desc, bool optional = false);

    /**
     * print the value of all params to an ostream
     */
    void printParams(std::ostream& os);

    //! return the banner string
    const std::string& getBanner() const { return _banner; }
    void setBanner(const std::string& banner);

    /**
     * print the help
     */
    void printHelp(std::ostream& os);

  protected:
    std::vector<CommandArgument> _args;
    std::vector<CommandArgument> _leftOvers;
    std::vector<CommandArgument> _leftOversOptional;
    std::string _banner;
    std::string _progName;

    const char* type2str(int t) const;
    void str2arg(const std::string& input, CommandArgument& ca) const;
    std::string arg2str(const CommandArgument& ca) const;



    /**
     * convert a string into an other type.
     */
    template<typename T>
    bool convertString(const std::string& s, T& x) const
    {
      std::istringstream i(s);
      if (! (i >> x))
        return false;
      return true;
    }

    std::string trim(const std::string& s) const;
};

} // end namespace

#endif
