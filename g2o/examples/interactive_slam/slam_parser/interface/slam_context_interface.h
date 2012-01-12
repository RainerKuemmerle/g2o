#ifndef SLAM_CONTEXT_INTERFACE_H
#define SLAM_CONTEXT_INTERFACE_H

#include "slam_parser/parser/slam_context.h"

namespace SlamParser {

  class AbstractSlamInterface;

  class SlamContextInterface : public SlamContext
  {
    public:
      SlamContextInterface(AbstractSlamInterface* slam);
      ~SlamContextInterface();

      bool process(CommandNode* commandNode);

    protected:
      AbstractSlamInterface* _slam;
  };

} // end namespace

#endif
