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

#include "draw_helpers.h"

#include "g2o/stuff/misc.h"

#include <cmath>

#ifdef __APPLE__
# include <OpenGL/glu.h>
#else
# include <GL/glu.h>
#endif

namespace g2o {

  /**
   * \brief handle the GLU quadratic
   */
  namespace {
    class GLUWrapper
    {
      public:
        static GLUquadricObj* getQuadradic()
        {
          static GLUWrapper inst;
          return inst._quadratic;
        }
      protected:
        GLUWrapper()
        {
          _quadratic = gluNewQuadric();              // Create A Pointer To The Quadric Object ( NEW )
          gluQuadricNormals(_quadratic, GLU_SMOOTH); // Create Smooth Normals ( NEW )
        }
        ~GLUWrapper()
        {
          gluDeleteQuadric(_quadratic);
        }
        GLUquadricObj* _quadratic;;
    };
  }

  void drawDisk(GLfloat radius)
  {
    gluDisk(GLUWrapper::getQuadradic(), 0, radius, 32, 1);
  }

  void drawCircle(GLfloat radius, int segments)
  {
    double angleStep = (2 * M_PI / (segments));
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= segments; i++) {
      double angle = i * angleStep;
      float x = radius * cos(angle);
      float y = radius * sin(angle);
      glVertex3f(x, y, 0.f);
    }
    glEnd();
  }

} // end namespace g2o
