// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// This file is part of g2o.
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with g2o.  If not, see <http://www.gnu.org/licenses/>.

#include "draw_helpers.h"

#include <cmath>

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
