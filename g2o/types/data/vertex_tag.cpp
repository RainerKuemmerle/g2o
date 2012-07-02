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

#include "vertex_tag.h"

#include "g2o/stuff/macros.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif

#ifdef G2O_HAVE_GLUT
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/freeglut.h>
#endif
#endif

#include <iomanip>
using namespace std;

namespace g2o {

  VertexTag::VertexTag() : RobotData()
  {
  }

  VertexTag::~VertexTag()
  {
  }

  bool VertexTag::read(std::istream& is)
  {
    is >> _name;
    is >> _position.x() >> _position.y() >> _position.z();
    is >> _odom2d.x() >> _odom2d.y() >> _odom2d.z();
    is >> _timestamp;
    is >> _hostname;
    is >> _loggerTimestamp;
    return true;
  }

  bool VertexTag::write(std::ostream& os) const
  {
    os << _name << " ";
    os << FIXED(_position.x() << " " << _position.y() << " " << _position.z() << " ");
    os << FIXED(_odom2d.x() << " " << _odom2d.y() << " " << _odom2d.z() << " ");
    os << FIXED(" " << timestamp() << " " << hostname() << " " << loggerTimestamp());
    return os.good();
  }



#ifdef G2O_HAVE_OPENGL
  VertexTagDrawAction::VertexTagDrawAction(): DrawAction(typeid(VertexTag).name()){
  }

  bool VertexTagDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (!DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _textSize = _previousParams->makeProperty<IntProperty>(_typeName + "::TEXT_SIZE", 1);
    } else {
      _textSize = 0;
    }
    return true;
  }

  HyperGraphElementAction* VertexTagDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                 HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;

    refreshPropertyPtrs(params_);
    if (! _previousParams){
      return this;
    }
    VertexTag* that = static_cast<VertexTag*>(element);

    glPushMatrix();
    glColor3f(1.,0.2,1.);
    glTranslatef(that->position().x(), that->position().y(), that->position().z());
    int textSize = 1;
    if (_textSize )
      textSize = _textSize->value();
#ifdef G2O_HAVE_GLUT
    glutSolidCube(0.1*textSize);
    glTranslatef(0.2*textSize, 0, 0);
    glScalef(0.003*textSize,0.003*textSize,1);
#ifndef __APPLE__
    glutStrokeString(GLUT_STROKE_ROMAN, (const unsigned char*)that->name().c_str());
#endif
#endif
    glPopMatrix();
    return this;
  }
#endif

}
