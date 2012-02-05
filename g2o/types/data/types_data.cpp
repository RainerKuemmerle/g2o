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

#include "g2o/config.h"

#include "robot_laser.h"

#include "g2o/core/factory.h"
#include "g2o/core/creators.h"
#include "g2o/stuff/macros.h"

namespace g2o {

  namespace types_data {
    int initialized = 0;

    void init()
    {
      if (types_data::initialized)
        return;
      Factory* factory = Factory::instance();
      //cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << endl;

      factory->registerType("ROBOTLASER1", new HyperGraphElementCreator<RobotLaser>);

#ifdef G2O_HAVE_OPENGL
      HyperGraphActionLibrary* actionLib = HyperGraphActionLibrary::instance();
      actionLib->registerAction(new RobotLaserDrawAction);
#endif

      types_data::initialized = 1;
    }
  }

  G2O_ATTRIBUTE_CONSTRUCTOR(init_types_data)
  {
    types_data::init();
  }

} // end namespace
