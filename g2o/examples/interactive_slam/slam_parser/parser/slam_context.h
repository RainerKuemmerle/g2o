#ifndef SLAM_CONTEXT_H
#define SLAM_CONTEXT_H

#include <vector>

namespace SlamParser {

class CommandNode;

class SlamContext
{
  public:
    SlamContext();
    virtual ~SlamContext();

    virtual bool process(CommandNode* commandNode);
};

} // end namespace

#endif
