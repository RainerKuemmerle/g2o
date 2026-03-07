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

#include "opengl_interface.h"

namespace g2o::opengl {

void drawArrow2D(float len, float head_width, float head_len) {
  ::g2o::opengl::begin_lines();
  ::g2o::opengl::vertex2f(0.F, 0.F);
  ::g2o::opengl::vertex2f(len, 0.F);
  ::g2o::opengl::end();

  ::g2o::opengl::normal3f(0.F, 0.F, 1.F);
  ::g2o::opengl::begin_triangles();
  ::g2o::opengl::vertex2f(len, 0.F);
  ::g2o::opengl::vertex2f(len - head_len, 0.5F * head_width);
  ::g2o::opengl::vertex2f(len - head_len, -0.5F * head_width);
  ::g2o::opengl::end();
}

void drawPoseBox() {
  ::g2o::opengl::push_matrix();
  ::g2o::opengl::scalef(0.5F, 1.F, 1.F);
  ::g2o::opengl::push_matrix();
  ::g2o::opengl::scalef(1.F, 0.25F, 0.5F);
  ::g2o::opengl::translatef(-0.5F, 0.5F, 0.F);
  ::g2o::opengl::color3f(1.0F, 0.3F, 0.3F);
  drawBox(1.F, 1.F, 1.F);
  ::g2o::opengl::pop_matrix();

  ::g2o::opengl::push_matrix();
  ::g2o::opengl::scalef(1.F, 0.25F, 0.5F);
  ::g2o::opengl::translatef(-0.5F, -0.5F, 0.F);
  ::g2o::opengl::color3f(1.0F, 0.1F, 0.1F);
  drawBox(1.F, 1.F, 1.F);
  ::g2o::opengl::pop_matrix();

  ::g2o::opengl::push_matrix();
  ::g2o::opengl::scalef(1.F, 0.25F, 0.5F);
  ::g2o::opengl::translatef(+0.5F, 0.5F, 0.F);
  ::g2o::opengl::color3f(0.3F, 0.3F, 1.0F);
  drawBox(1.F, 1.F, 1.F);
  ::g2o::opengl::pop_matrix();

  ::g2o::opengl::push_matrix();
  ::g2o::opengl::scalef(1.F, 0.25F, 0.5F);
  ::g2o::opengl::translatef(+0.5F, -0.5F, 0.F);
  ::g2o::opengl::color3f(0.1F, 0.1F, 1.F);
  drawBox(1.F, 1.F, 1.F);
  ::g2o::opengl::pop_matrix();
  ::g2o::opengl::pop_matrix();
}

void drawBox(float l, float w, float h) {
  float sx = l * 0.5F;
  float sy = w * 0.5F;
  float sz = h * 0.5F;

  ::g2o::opengl::begin_quads();
  // bottom
  ::g2o::opengl::normal3f(0.0F, 0.0F, -1.0F);
  ::g2o::opengl::vertex3f(-sx, -sy, -sz);
  ::g2o::opengl::vertex3f(-sx, sy, -sz);
  ::g2o::opengl::vertex3f(sx, sy, -sz);
  ::g2o::opengl::vertex3f(sx, -sy, -sz);
  // top
  ::g2o::opengl::normal3f(0.0F, 0.0F, 1.0F);
  ::g2o::opengl::vertex3f(-sx, -sy, sz);
  ::g2o::opengl::vertex3f(-sx, sy, sz);
  ::g2o::opengl::vertex3f(sx, sy, sz);
  ::g2o::opengl::vertex3f(sx, -sy, sz);
  // back
  ::g2o::opengl::normal3f(-1.0F, 0.0F, 0.0F);
  ::g2o::opengl::vertex3f(-sx, -sy, -sz);
  ::g2o::opengl::vertex3f(-sx, sy, -sz);
  ::g2o::opengl::vertex3f(-sx, sy, sz);
  ::g2o::opengl::vertex3f(-sx, -sy, sz);
  // front
  ::g2o::opengl::normal3f(1.0F, 0.0F, 0.0F);
  ::g2o::opengl::vertex3f(sx, -sy, -sz);
  ::g2o::opengl::vertex3f(sx, sy, -sz);
  ::g2o::opengl::vertex3f(sx, sy, sz);
  ::g2o::opengl::vertex3f(sx, -sy, sz);
  // left
  ::g2o::opengl::normal3f(0.0F, -1.0F, 0.0F);
  ::g2o::opengl::vertex3f(-sx, -sy, -sz);
  ::g2o::opengl::vertex3f(sx, -sy, -sz);
  ::g2o::opengl::vertex3f(sx, -sy, sz);
  ::g2o::opengl::vertex3f(-sx, -sy, sz);
  // right
  ::g2o::opengl::normal3f(0.0F, 1.0F, 0.0F);
  ::g2o::opengl::vertex3f(-sx, sy, -sz);
  ::g2o::opengl::vertex3f(sx, sy, -sz);
  ::g2o::opengl::vertex3f(sx, sy, sz);
  ::g2o::opengl::vertex3f(-sx, sy, sz);
  ::g2o::opengl::end();
}

void drawPlane(float l, float w) {
  float sx = l * 0.5F;
  float sy = w * 0.5F;
  ::g2o::opengl::begin_quads();
  ::g2o::opengl::normal3f(0.0F, 0.0F, 1.0F);
  ::g2o::opengl::vertex3f(-sx, -sy, 0.F);
  ::g2o::opengl::vertex3f(-sx, sy, 0.F);
  ::g2o::opengl::vertex3f(sx, sy, 0.F);
  ::g2o::opengl::vertex3f(sx, -sy, 0.F);
  ::g2o::opengl::end();
}

void drawSphere(float radius) { ::g2o::opengl::draw_sphere(radius); }

void drawEllipsoid(float r1, float r2, float r3) {
  bool hasNormalization = ::g2o::opengl::is_enabled(Capability::NORMALIZE);
  if (!hasNormalization) ::g2o::opengl::enable(Capability::NORMALIZE);
  ::g2o::opengl::push_matrix();
  ::g2o::opengl::scalef(r1, r2, r3);
  ::g2o::opengl::draw_sphere(1.0F);
  ::g2o::opengl::pop_matrix();
  if (!hasNormalization) ::g2o::opengl::disable(Capability::NORMALIZE);
}

void drawCone(float radius, float height) {
  ::g2o::opengl::push_matrix();
  ::g2o::opengl::rotatef(-90.F, 1.F, 0.F, 0.F);
  ::g2o::opengl::translatef(0.F, 0.F, -height / 2.0F);
  // approximate cone with a tapered cylinder fallback
  ::g2o::opengl::draw_cylinder(radius, height);
  ::g2o::opengl::pop_matrix();
}

void drawCylinder(float radius, float height) {
  ::g2o::opengl::push_matrix();
  ::g2o::opengl::rotatef(-90, 1.F, 0.F, 0.F);
  ::g2o::opengl::translatef(0.F, 0.F, +height / 2.0F);
  ::g2o::opengl::draw_disk(radius);
  ::g2o::opengl::translatef(0, 0, -height);
  ::g2o::opengl::draw_cylinder(radius, height);
  ::g2o::opengl::rotatef(180, 1.F, 0.F, 0.F);
  ::g2o::opengl::draw_disk(radius);
  ::g2o::opengl::pop_matrix();
}

void drawDisk(float radius) {
  ::g2o::opengl::rotatef(90, 0.F, 1.F, 0.F);
  ::g2o::opengl::draw_disk(radius);
}

void drawCircle(float radius, int segments) {
  double angleStep = (2 * M_PI / (segments));
  ::g2o::opengl::begin_line_strip();
  for (int i = 0; i <= segments; i++) {
    double angle = i * angleStep;
    float x = radius * cos(angle);
    float y = radius * sin(angle);
    ::g2o::opengl::vertex3f(x, y, 0.F);
  }
  ::g2o::opengl::end();
}

void drawPyramid(float length, float height) {
  ::g2o::opengl::push_matrix();
  ::g2o::opengl::translatef(0.F, 0.F, -height / 2.0F);
  ::g2o::opengl::rotatef(45, 0.F, 0.F, 1.F);
  ::g2o::opengl::draw_cylinder(length, height);
  ::g2o::opengl::draw_disk(length);
  ::g2o::opengl::pop_matrix();
}

void drawRangeRing(float range, float fov, float range_width) {
  ::g2o::opengl::push_matrix();
  ::g2o::opengl::rotatef((fov / 2.0F) - 90, 0.F, 0.F, 1.F);
  // approximate partial disk with scaled full disk for fallback
  ::g2o::opengl::draw_disk(range + range_width);
  ::g2o::opengl::pop_matrix();
}

void drawSlice(float radius, float height, float fov, int slices_per_circle) {
  double fov_rad = fov / 180. * M_PI;  // convert to rad
  int num_slices =
      static_cast<int>(slices_per_circle * (fov_rad / (2 * M_PI))) + 1;
  double angle_step = fov_rad / num_slices;
  double angle_step_half = angle_step * 0.5;

  float height_half = height * 0.5F;
  float lower_z = -height_half;
  float upper_z = height_half;

  auto last_x = static_cast<float>(std::cos(-fov_rad * 0.5F) * radius);
  auto last_y = static_cast<float>(std::sin(-fov_rad * 0.5F) * radius);

  ::g2o::opengl::push_matrix();
  ::g2o::opengl::begin_triangles();
  ::g2o::opengl::normal3f(static_cast<float>(std::sin(-fov_rad * 0.5)),
                          static_cast<float>(-std::cos(-fov_rad * 0.5)), 0.F);
  ::g2o::opengl::vertex3f(0.F, 0.F, upper_z);
  ::g2o::opengl::vertex3f(0.F, 0.F, lower_z);
  ::g2o::opengl::vertex3f(last_x, last_y, upper_z);
  ::g2o::opengl::vertex3f(last_x, last_y, upper_z);
  ::g2o::opengl::vertex3f(last_x, last_y, lower_z);
  ::g2o::opengl::vertex3f(0.F, 0.F, lower_z);

  double start_angle = (-0.5 * fov_rad) + angle_step;
  double angle = start_angle;
  for (int i = 0; i < num_slices; ++i) {
    auto x = static_cast<float>(std::cos(angle) * radius);
    auto y = static_cast<float>(std::sin(angle) * radius);
    auto front_normal_x = static_cast<float>(std::cos(angle + angle_step_half));
    auto front_normal_y = static_cast<float>(std::sin(angle + angle_step_half));

    // lower triangle
    ::g2o::opengl::normal3f(0.F, 0.F, -1.F);
    ::g2o::opengl::vertex3f(0.F, 0.F, lower_z);
    ::g2o::opengl::vertex3f(x, y, lower_z);
    ::g2o::opengl::vertex3f(last_x, last_y, lower_z);
    // upper
    ::g2o::opengl::normal3f(0.F, 0.F, 1.F);
    ::g2o::opengl::vertex3f(0.F, 0.F, upper_z);
    ::g2o::opengl::vertex3f(x, y, upper_z);
    ::g2o::opengl::vertex3f(last_x, last_y, upper_z);
    // front rectangle (we use two triangles)
    ::g2o::opengl::normal3f(front_normal_x, front_normal_y, 0.F);
    ::g2o::opengl::vertex3f(last_x, last_y, upper_z);
    ::g2o::opengl::vertex3f(last_x, last_y, lower_z);
    ::g2o::opengl::vertex3f(x, y, upper_z);
    ::g2o::opengl::vertex3f(x, y, upper_z);
    ::g2o::opengl::vertex3f(x, y, lower_z);
    ::g2o::opengl::vertex3f(last_x, last_y, lower_z);

    last_x = x;
    last_y = y;
    angle += angle_step;
  }

  ::g2o::opengl::normal3f(static_cast<float>(-std::sin(fov_rad * 0.5)),
                          static_cast<float>(std::cos(fov_rad * 0.5)), -0.F);
  ::g2o::opengl::vertex3f(0.F, 0.F, upper_z);
  ::g2o::opengl::vertex3f(0.F, 0.F, lower_z);
  ::g2o::opengl::vertex3f(last_x, last_y, upper_z);
  ::g2o::opengl::vertex3f(last_x, last_y, upper_z);
  ::g2o::opengl::vertex3f(last_x, last_y, lower_z);
  ::g2o::opengl::vertex3f(0.F, 0.F, lower_z);

  ::g2o::opengl::end();
  ::g2o::opengl::pop_matrix();
}

void drawPoint(float pointSize) {
  ::g2o::opengl::point_size(pointSize);
  ::g2o::opengl::begin_points();
  ::g2o::opengl::vertex3f(0, 0, 0);
  ::g2o::opengl::end();
}

}  // namespace g2o::opengl
