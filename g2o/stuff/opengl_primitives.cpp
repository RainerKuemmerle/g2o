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

#include "opengl_primitives.h"

#include <cstdlib>
#include <cmath>

#ifdef __APPLE__
# include <OpenGL/glu.h>
#else
# include <GL/glu.h>
#endif

namespace g2o {
namespace opengl {

/**
 * \brief handle the GLU quadratic
 */
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
      //std::cerr << __PRETTY_FUNCTION__ << std::endl;
      _quadratic = gluNewQuadric();              // Create A Pointer To The Quadric Object ( NEW )
      gluQuadricNormals(_quadratic, GLU_SMOOTH); // Create Smooth Normals ( NEW )
    }
    ~GLUWrapper()
    {
      //std::cerr << __PRETTY_FUNCTION__ << std::endl;
      gluDeleteQuadric(_quadratic);
    }
    GLUquadricObj* _quadratic;;
};

void drawArrow2D(float len, float head_width, float head_len)
{
  glBegin(GL_LINES);
  glVertex2f(0.f, 0.f);
  glVertex2f(len, 0.f);
  glEnd();

  glNormal3f(0.f,0.f,1.f);
  glBegin(GL_TRIANGLES);
  glVertex2f(len, 0.f);
  glVertex2f(len - head_len,  0.5f*head_width);
  glVertex2f(len - head_len, -0.5f*head_width);
  glEnd();
}

void drawPoseBox()
{
  glPushMatrix();
  glScalef(0.5f,1.f,1.f);
  glPushMatrix();
  glScalef(1.f,0.25f,0.5f);
  glTranslatef(-0.5f,0.5f,0.f);
  glColor3f(1.0f, 0.3f, 0.3f);
  drawBox(1.f, 1.f, 1.f);
  glPopMatrix();

  glPushMatrix();
  glScalef(1.f,0.25f,0.5f);
  glTranslatef(-0.5f,-0.5f,0.f);
  glColor3f(1.0f, 0.1f, 0.1f);
  drawBox(1.f, 1.f, 1.f);
  glPopMatrix();

  glPushMatrix();
  glScalef(1.f,0.25f,0.5f);
  glTranslatef(+0.5f,0.5f,0.f);
  glColor3f(0.3f, 0.3f, 1.0f);
  drawBox(1.f, 1.f, 1.f);
  glPopMatrix();

  glPushMatrix();
  glScalef(1.f,0.25f,0.5f);
  glTranslatef(+0.5f,-0.5f,0.f);
  glColor3f(0.1f, 0.1f, 1.f);
  drawBox(1.f, 1.f, 1.f);
  glPopMatrix();
  glPopMatrix();
}

void drawBox(GLfloat l, GLfloat w, GLfloat h)
{
  GLfloat sx = l*0.5f;
  GLfloat sy = w*0.5f;
  GLfloat sz = h*0.5f;

  glBegin(GL_QUADS);
  // bottom
  glNormal3f( 0.0f, 0.0f,-1.0f);
  glVertex3f(-sx, -sy, -sz);
  glVertex3f(-sx, sy, -sz);
  glVertex3f(sx, sy, -sz);
  glVertex3f(sx, -sy, -sz);
  // top
  glNormal3f( 0.0f, 0.0f,1.0f);
  glVertex3f(-sx, -sy, sz);
  glVertex3f(-sx, sy, sz);
  glVertex3f(sx, sy, sz);
  glVertex3f(sx, -sy, sz);
  // back
  glNormal3f(-1.0f, 0.0f, 0.0f);
  glVertex3f(-sx, -sy, -sz);
  glVertex3f(-sx, sy, -sz);
  glVertex3f(-sx, sy, sz);
  glVertex3f(-sx, -sy, sz);
  // front
  glNormal3f( 1.0f, 0.0f, 0.0f);
  glVertex3f(sx, -sy, -sz);
  glVertex3f(sx, sy, -sz);
  glVertex3f(sx, sy, sz);
  glVertex3f(sx, -sy, sz);
  // left
  glNormal3f( 0.0f, -1.0f, 0.0f);
  glVertex3f(-sx, -sy, -sz);
  glVertex3f(sx, -sy, -sz);
  glVertex3f(sx, -sy, sz);
  glVertex3f(-sx, -sy, sz);
  //right
  glNormal3f( 0.0f, 1.0f, 0.0f);
  glVertex3f(-sx, sy, -sz);
  glVertex3f(sx, sy, -sz);
  glVertex3f(sx, sy, sz);
  glVertex3f(-sx, sy, sz);
  glEnd();
}

void drawPlane(GLfloat l, GLfloat w)
{
  GLfloat sx = l*0.5f;
  GLfloat sy = w*0.5f;

  glBegin(GL_QUADS);
  glNormal3f( 0.0f, 0.0f, 1.0f);
  glVertex3f(-sx, -sy, 0.f);
  glVertex3f(-sx, sy, 0.f);
  glVertex3f(sx, sy, 0.f);
  glVertex3f(sx, -sy, 0.f);
  glEnd();
}

void drawSphere(GLfloat radius)
{
  gluSphere(GLUWrapper::getQuadradic(), radius, 32, 32);
}

void drawEllipsoid(GLfloat r1, GLfloat r2, GLfloat r3)
{
  GLboolean hasNormalization = glIsEnabled(GL_NORMALIZE);
  if (!hasNormalization)
    glEnable(GL_NORMALIZE);
  glPushMatrix();
  glScalef(r1, r2, r3);
  gluSphere(GLUWrapper::getQuadradic(), 1.0f, 32, 32);
  glPopMatrix();
  if (!hasNormalization)
    glDisable(GL_NORMALIZE);
}

void drawCone(GLfloat radius, GLfloat height)
{
  glPushMatrix();
  glRotatef(-90.f, 1.f, 0.f, 0.f);
  glTranslatef(0.f, 0.f, - height/2.0f);
  gluCylinder(GLUWrapper::getQuadradic(), radius, 0.f, height, 32, 1);
  gluDisk(GLUWrapper::getQuadradic(), 0, radius, 32, 1);
  glPopMatrix();
}

void drawCylinder(GLfloat radius, GLfloat height)
{
  glPushMatrix();
  glRotatef(-90, 1.f, 0.f, 0.f);
  glTranslatef(0.f, 0.f, + height/2.0f);
  gluDisk(GLUWrapper::getQuadradic(), 0.f, radius, 32, 1);
  glTranslatef(0, 0, - height);
  gluCylinder(GLUWrapper::getQuadradic(), radius, radius, height, 32, 1);
  glRotatef(180, 1.f, 0.f, 0.f);
  gluDisk(GLUWrapper::getQuadradic(), 0, radius, 32, 1);
  glPopMatrix();
}

void drawDisk(GLfloat radius)
{
  glRotatef(90, 0.f, 1.f, 0.f);
  gluDisk(GLUWrapper::getQuadradic(), 0, radius, 32, 1);
}

void drawPyramid(GLfloat length, GLfloat height)
{
  glPushMatrix();
  glTranslatef(0.f, 0.f, - height/2.0f);
  glRotatef(45, 0.f, 0.f, 1.f);
  gluCylinder(GLUWrapper::getQuadradic(), length, 0.f, height, 32, 1);
  gluDisk(GLUWrapper::getQuadradic(), 0, length, 32, 1);
  glPopMatrix();
}

void drawRangeRing(GLfloat range, GLfloat fov, GLfloat range_width)
{
  glPushMatrix();
  glRotatef((fov/2.0f) - 90, 0.f, 0.f, 1.f);
  gluPartialDisk(GLUWrapper::getQuadradic(), range, range + range_width, 32, 1, 0.f, fov);
  glPopMatrix();
}

void drawSlice(GLfloat radius, GLfloat height, GLfloat fov, int slices_per_circle)
{
  double fov_rad = fov/180.*M_PI; // convert to rad
  int num_slices = int(slices_per_circle * (fov_rad / (2*M_PI))) + 1;
  double angle_step = fov_rad / num_slices;
  double angle_step_half = angle_step * 0.5;

  GLfloat height_half = height * 0.5f;
  GLfloat lower_z = -height_half;
  GLfloat upper_z =  height_half;

  GLfloat last_x = float(std::cos(-fov_rad * 0.5f) * radius);
  GLfloat last_y = float(std::sin(-fov_rad * 0.5f) * radius);

  glPushMatrix();
  glBegin(GL_TRIANGLES);
  glNormal3f((float)std::sin(-fov_rad * 0.5), (float)-std::cos(-fov_rad * 0.5), 0.f);
  glVertex3f(0.f, 0.f, upper_z);
  glVertex3f(0.f, 0.f, lower_z);
  glVertex3f(last_x, last_y, upper_z);
  glVertex3f(last_x, last_y, upper_z);
  glVertex3f(last_x, last_y, lower_z);
  glVertex3f(0.f, 0.f, lower_z);

  double start_angle = -0.5*fov_rad + angle_step;
  double angle       = start_angle;
  for (int i = 0; i < num_slices; ++i) {
    GLfloat x = float(std::cos(angle) * radius);
    GLfloat y = float(std::sin(angle) * radius);
    GLfloat front_normal_x = (float)std::cos(angle + angle_step_half);
    GLfloat front_normal_y = (float)std::sin(angle + angle_step_half);

    // lower triangle
    glNormal3f(0.f, 0.f, -1.f);
    glVertex3f(0.f, 0.f, lower_z);
    glVertex3f(x, y, lower_z);
    glVertex3f(last_x, last_y, lower_z);
    // upper
    glNormal3f(0.f, 0.f, 1.f);
    glVertex3f(0.f, 0.f, upper_z);
    glVertex3f(x, y, upper_z);
    glVertex3f(last_x, last_y, upper_z);
    //front rectangle (we use two triangles)
    glNormal3f(front_normal_x, front_normal_y, 0.f);
    glVertex3f(last_x, last_y, upper_z);
    glVertex3f(last_x, last_y, lower_z);
    glVertex3f(x, y, upper_z);
    glVertex3f(x, y, upper_z);
    glVertex3f(x, y, lower_z);
    glVertex3f(last_x, last_y, lower_z);

    last_x = x;
    last_y = y;
    angle += angle_step;
  }

  glNormal3f(float(-std::sin(fov_rad * 0.5)), float(std::cos(fov_rad * 0.5)), -0.f);
  glVertex3f(0.f, 0.f, upper_z);
  glVertex3f(0.f, 0.f, lower_z);
  glVertex3f(last_x, last_y, upper_z);
  glVertex3f(last_x, last_y, upper_z);
  glVertex3f(last_x, last_y, lower_z);
  glVertex3f(0.f, 0.f, lower_z);

  glEnd();
  glPopMatrix();
}

} // end namespace
} // end namespace
