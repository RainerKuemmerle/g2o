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

#include <cmath>
#include <cstdlib>

#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

namespace g2o {
namespace opengl {

/**
 * \brief handle the GLU quadratic
 */
class GLUWrapper {
 public:
  static GLUquadricObj* getQuadradic() {
    static GLUWrapper inst;
    return inst.quadratic_;
  }

 protected:
  GLUWrapper() {
    // std::cerr << __PRETTY_FUNCTION__ << std::endl;
    quadratic_ =
        gluNewQuadric();  // Create A Pointer To The Quadric Object ( NEW )
    gluQuadricNormals(quadratic_, GLU_SMOOTH);  // Create Smooth Normals ( NEW )
  }
  ~GLUWrapper() {
    // std::cerr << __PRETTY_FUNCTION__ << std::endl;
    gluDeleteQuadric(quadratic_);
  }
  GLUquadricObj* quadratic_;
  ;
};

void drawArrow2D(float len, float head_width, float head_len) {
  glBegin(GL_LINES);
  glVertex2f(0.F, 0.F);
  glVertex2f(len, 0.F);
  glEnd();

  glNormal3f(0.F, 0.F, 1.F);
  glBegin(GL_TRIANGLES);
  glVertex2f(len, 0.F);
  glVertex2f(len - head_len, 0.5F * head_width);
  glVertex2f(len - head_len, -0.5F * head_width);
  glEnd();
}

void drawPoseBox() {
  glPushMatrix();
  glScalef(0.5F, 1.F, 1.F);
  glPushMatrix();
  glScalef(1.F, 0.25F, 0.5F);
  glTranslatef(-0.5F, 0.5F, 0.F);
  glColor3f(1.0F, 0.3F, 0.3F);
  drawBox(1.F, 1.F, 1.F);
  glPopMatrix();

  glPushMatrix();
  glScalef(1.F, 0.25F, 0.5F);
  glTranslatef(-0.5F, -0.5F, 0.F);
  glColor3f(1.0F, 0.1F, 0.1F);
  drawBox(1.F, 1.F, 1.F);
  glPopMatrix();

  glPushMatrix();
  glScalef(1.F, 0.25F, 0.5F);
  glTranslatef(+0.5F, 0.5F, 0.F);
  glColor3f(0.3F, 0.3F, 1.0F);
  drawBox(1.F, 1.F, 1.F);
  glPopMatrix();

  glPushMatrix();
  glScalef(1.F, 0.25F, 0.5F);
  glTranslatef(+0.5F, -0.5F, 0.F);
  glColor3f(0.1F, 0.1F, 1.F);
  drawBox(1.F, 1.F, 1.F);
  glPopMatrix();
  glPopMatrix();
}

void drawBox(GLfloat l, GLfloat w, GLfloat h) {
  GLfloat sx = l * 0.5F;
  GLfloat sy = w * 0.5F;
  GLfloat sz = h * 0.5F;

  glBegin(GL_QUADS);
  // bottom
  glNormal3f(0.0F, 0.0F, -1.0F);
  glVertex3f(-sx, -sy, -sz);
  glVertex3f(-sx, sy, -sz);
  glVertex3f(sx, sy, -sz);
  glVertex3f(sx, -sy, -sz);
  // top
  glNormal3f(0.0F, 0.0F, 1.0F);
  glVertex3f(-sx, -sy, sz);
  glVertex3f(-sx, sy, sz);
  glVertex3f(sx, sy, sz);
  glVertex3f(sx, -sy, sz);
  // back
  glNormal3f(-1.0F, 0.0F, 0.0F);
  glVertex3f(-sx, -sy, -sz);
  glVertex3f(-sx, sy, -sz);
  glVertex3f(-sx, sy, sz);
  glVertex3f(-sx, -sy, sz);
  // front
  glNormal3f(1.0F, 0.0F, 0.0F);
  glVertex3f(sx, -sy, -sz);
  glVertex3f(sx, sy, -sz);
  glVertex3f(sx, sy, sz);
  glVertex3f(sx, -sy, sz);
  // left
  glNormal3f(0.0F, -1.0F, 0.0F);
  glVertex3f(-sx, -sy, -sz);
  glVertex3f(sx, -sy, -sz);
  glVertex3f(sx, -sy, sz);
  glVertex3f(-sx, -sy, sz);
  // right
  glNormal3f(0.0F, 1.0F, 0.0F);
  glVertex3f(-sx, sy, -sz);
  glVertex3f(sx, sy, -sz);
  glVertex3f(sx, sy, sz);
  glVertex3f(-sx, sy, sz);
  glEnd();
}

void drawPlane(GLfloat l, GLfloat w) {
  GLfloat sx = l * 0.5F;
  GLfloat sy = w * 0.5F;

  glBegin(GL_QUADS);
  glNormal3f(0.0F, 0.0F, 1.0F);
  glVertex3f(-sx, -sy, 0.F);
  glVertex3f(-sx, sy, 0.F);
  glVertex3f(sx, sy, 0.F);
  glVertex3f(sx, -sy, 0.F);
  glEnd();
}

void drawSphere(GLfloat radius) {
  gluSphere(GLUWrapper::getQuadradic(), radius, 32, 32);
}

void drawEllipsoid(GLfloat r1, GLfloat r2, GLfloat r3) {
  GLboolean hasNormalization = glIsEnabled(GL_NORMALIZE);
  if (!hasNormalization) glEnable(GL_NORMALIZE);
  glPushMatrix();
  glScalef(r1, r2, r3);
  gluSphere(GLUWrapper::getQuadradic(), 1.0F, 32, 32);
  glPopMatrix();
  if (!hasNormalization) glDisable(GL_NORMALIZE);
}

void drawCone(GLfloat radius, GLfloat height) {
  glPushMatrix();
  glRotatef(-90.F, 1.F, 0.F, 0.F);
  glTranslatef(0.F, 0.F, -height / 2.0F);
  gluCylinder(GLUWrapper::getQuadradic(), radius, 0.F, height, 32, 1);
  gluDisk(GLUWrapper::getQuadradic(), 0, radius, 32, 1);
  glPopMatrix();
}

void drawCylinder(GLfloat radius, GLfloat height) {
  glPushMatrix();
  glRotatef(-90, 1.F, 0.F, 0.F);
  glTranslatef(0.F, 0.F, +height / 2.0F);
  gluDisk(GLUWrapper::getQuadradic(), 0.F, radius, 32, 1);
  glTranslatef(0, 0, -height);
  gluCylinder(GLUWrapper::getQuadradic(), radius, radius, height, 32, 1);
  glRotatef(180, 1.F, 0.F, 0.F);
  gluDisk(GLUWrapper::getQuadradic(), 0, radius, 32, 1);
  glPopMatrix();
}

void drawDisk(GLfloat radius) {
  glRotatef(90, 0.F, 1.F, 0.F);
  gluDisk(GLUWrapper::getQuadradic(), 0, radius, 32, 1);
}

void drawCircle(GLfloat radius, int segments) {
  double angleStep = (2 * M_PI / (segments));
  glBegin(GL_LINE_STRIP);
  for (int i = 0; i <= segments; i++) {
    double angle = i * angleStep;
    float x = radius * cos(angle);
    float y = radius * sin(angle);
    glVertex3f(x, y, 0.F);
  }
  glEnd();
}

void drawPyramid(GLfloat length, GLfloat height) {
  glPushMatrix();
  glTranslatef(0.F, 0.F, -height / 2.0F);
  glRotatef(45, 0.F, 0.F, 1.F);
  gluCylinder(GLUWrapper::getQuadradic(), length, 0.F, height, 4, 1);
  gluDisk(GLUWrapper::getQuadradic(), 0, length, 4, 1);
  glPopMatrix();
}

void drawRangeRing(GLfloat range, GLfloat fov, GLfloat range_width) {
  glPushMatrix();
  glRotatef((fov / 2.0F) - 90, 0.F, 0.F, 1.F);
  gluPartialDisk(GLUWrapper::getQuadradic(), range, range + range_width, 32, 1,
                 0.F, fov);
  glPopMatrix();
}

void drawSlice(GLfloat radius, GLfloat height, GLfloat fov,
               int slices_per_circle) {
  double fov_rad = fov / 180. * M_PI;  // convert to rad
  int num_slices = static_cast<int>(slices_per_circle * (fov_rad / (2 * M_PI))) + 1;
  double angle_step = fov_rad / num_slices;
  double angle_step_half = angle_step * 0.5;

  GLfloat height_half = height * 0.5F;
  GLfloat lower_z = -height_half;
  GLfloat upper_z = height_half;

  auto last_x = static_cast<float>(std::cos(-fov_rad * 0.5F) * radius);
  auto last_y = static_cast<float>(std::sin(-fov_rad * 0.5F) * radius);

  glPushMatrix();
  glBegin(GL_TRIANGLES);
  glNormal3f(static_cast<float>(std::sin(-fov_rad * 0.5)),
             static_cast<float>(-std::cos(-fov_rad * 0.5)), 0.F);
  glVertex3f(0.F, 0.F, upper_z);
  glVertex3f(0.F, 0.F, lower_z);
  glVertex3f(last_x, last_y, upper_z);
  glVertex3f(last_x, last_y, upper_z);
  glVertex3f(last_x, last_y, lower_z);
  glVertex3f(0.F, 0.F, lower_z);

  double start_angle = -0.5 * fov_rad + angle_step;
  double angle = start_angle;
  for (int i = 0; i < num_slices; ++i) {
    auto x = static_cast<float>(std::cos(angle) * radius);
    auto y = static_cast<float>(std::sin(angle) * radius);
    auto front_normal_x = static_cast<float>(std::cos(angle + angle_step_half));
    auto front_normal_y = static_cast<float>(std::sin(angle + angle_step_half));

    // lower triangle
    glNormal3f(0.F, 0.F, -1.F);
    glVertex3f(0.F, 0.F, lower_z);
    glVertex3f(x, y, lower_z);
    glVertex3f(last_x, last_y, lower_z);
    // upper
    glNormal3f(0.F, 0.F, 1.F);
    glVertex3f(0.F, 0.F, upper_z);
    glVertex3f(x, y, upper_z);
    glVertex3f(last_x, last_y, upper_z);
    // front rectangle (we use two triangles)
    glNormal3f(front_normal_x, front_normal_y, 0.F);
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

  glNormal3f(static_cast<float>(-std::sin(fov_rad * 0.5)),
             static_cast<float>(std::cos(fov_rad * 0.5)), -0.F);
  glVertex3f(0.F, 0.F, upper_z);
  glVertex3f(0.F, 0.F, lower_z);
  glVertex3f(last_x, last_y, upper_z);
  glVertex3f(last_x, last_y, upper_z);
  glVertex3f(last_x, last_y, lower_z);
  glVertex3f(0.F, 0.F, lower_z);

  glEnd();
  glPopMatrix();
}

void drawPoint(float pointSize) {
  glPointSize(pointSize);
  glBegin(GL_POINTS);
  glVertex3f(0, 0, 0);
  glEnd();
}
}  // namespace opengl
}  // namespace g2o
