#ifndef PARSER_INTERFACE_H
#define PARSER_INTERFACE_H

#include "abstract_slam_interface.h"

#include <iosfwd>
#include <sstream>

namespace SlamParser {

  class SlamContextInterface;
  class Driver;

  /**
   * \brief top-level interface to the parser
   */
  class ParserInterface
  {
    public:
      /**
       * construct a parser and use the given AbstractSlamInterface to communicate with the SLAM algorithm.
       */
      ParserInterface(AbstractSlamInterface* slamInterface);
      virtual ~ParserInterface();

      /**
       * parse a single command and forward to the SLAM engine
       */ 
      bool parseCommand(std::istream& input);

    protected:
      SlamContextInterface* _slamContextInterface;
      Driver* _driver;
      std::stringstream _buffer;
  };

} // end namespace

#endif
