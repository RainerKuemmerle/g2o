#include <iostream>

#include "slam_context.h"
#include "driver.h"
#include "commands.h"
using namespace std;

int main ()
{
  SlamParser::SlamContext slamContext;
  SlamParser::Driver driver(slamContext);
  driver.trace_parsing = true;
  driver.trace_scanning = true;

  bool parseStatus = driver.parse_stream(cin);
  if (! parseStatus)
    return 1;
}
