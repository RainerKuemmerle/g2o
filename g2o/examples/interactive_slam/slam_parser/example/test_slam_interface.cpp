#include <iostream>

#include "example_slam_interface.h"
#include "slam_parser/interface/parser_interface.h"
using namespace std;

int main()
{
  ExampleSlamInterface slamInterface;
  SlamParser::ParserInterface parserInterface(&slamInterface);

  while (parserInterface.parseCommand(cin))
  {
    // do something additional if needed
  }

  return 0;
}
