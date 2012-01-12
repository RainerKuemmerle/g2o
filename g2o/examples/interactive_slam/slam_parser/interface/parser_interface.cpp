#include "parser_interface.h"

#include "slam_context_interface.h"
#include "slam_parser/parser/driver.h"

namespace SlamParser {

  ParserInterface::ParserInterface(AbstractSlamInterface* slamInterface)
  {
    _slamContextInterface = new SlamContextInterface(slamInterface);
    _driver = new Driver(*_slamContextInterface);
    //_driver->trace_parsing = true;
    //_driver->trace_scanning = true;
  }

  ParserInterface::~ParserInterface()
  {
    delete _slamContextInterface;
    delete _driver;
  }

  bool ParserInterface::parseCommand(std::istream& input)
  {
    if (input.eof())
      return false;
    _buffer.str("");
    _buffer.clear();
    input.get(*_buffer.rdbuf(), ';');
    if (! input.eof()) // get the ';'
      _buffer << (char)input.get();
    _driver->parse_stream(_buffer);
    return true;
  }

} // end namespace
