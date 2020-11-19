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

#ifndef G2O_COMMAND_ARGS_H
#define G2O_COMMAND_ARGS_H

#include <string>
#include <vector>
#include <iostream>

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
    /** add an vector of doubles as a parameter */
    void param(const std::string& name, std::vector<double>& p, const std::vector<double>& defValue, const std::string& desc);
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

    /**
     * returns true, if the param was parsed via the command line
     */
    bool parsedParam(const std::string& paramFlag) const;

  protected:
    std::vector<CommandArgument> _args;
    std::vector<CommandArgument> _leftOvers;
    std::vector<CommandArgument> _leftOversOptional;
    std::string _banner;
    std::string _progName;

    const char* type2str(int t) const;
    void str2arg(const std::string& input, CommandArgument& ca) const;
    std::string arg2str(const CommandArgument& ca) const;
};

} // end namespace

#endif
