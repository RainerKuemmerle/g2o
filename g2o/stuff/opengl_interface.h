// Minimal OpenGL abstraction for g2o (small, incremental)
#ifndef G2O_OPENGL_INTERFACE_H
#define G2O_OPENGL_INTERFACE_H

#include <initializer_list>

#include "opengl_api.h"

namespace g2o::opengl {

// NOLINTBEGIN
enum class Capability : unsigned int {
  // Normalization of normals
  NORMALIZE = 0x0BA1,
  // Lighting
  LIGHTING = 0x0B50,
  // Blending
  BLEND = 0x0BE2,
  // Depth test
  DEPTH_TEST = 0x0B71,
  // Line smoothing
  LINE_SMOOTH = 0x0B20,
  // Attribute bits used with glPushAttrib / glPopAttrib
  ENABLE_BIT = 0x00002000,
  POINT_BIT = 0x00000002,
  COLOR_BUFFER_BIT = 0x00004000,
  DEPTH_BUFFER_BIT = 0x00000100,
  // Shading models
  FLAT = 0x1D00,
  SMOOTH = 0x1D01,
  // Blending factors
  SRC_ALPHA = 0x0302,
  ONE_MINUS_SRC_ALPHA = 0x0303
};
// NOLINTEND

// Initialize backend (optional)
void G2O_OPENGL_API init();

// Basic immediate-mode helpers used by current codebase
void G2O_OPENGL_API begin_triangles();
void G2O_OPENGL_API begin_quads();
void G2O_OPENGL_API begin_lines();
void G2O_OPENGL_API begin_line_strip();
void G2O_OPENGL_API begin_points();
void G2O_OPENGL_API end();
void G2O_OPENGL_API vertex3f(float x, float y, float z);
void G2O_OPENGL_API vertex2f(float x, float y);
void G2O_OPENGL_API color3f(float r, float g, float b);
void G2O_OPENGL_API normal3f(float x, float y, float z);
void G2O_OPENGL_API push_matrix();
void G2O_OPENGL_API pop_matrix();
void G2O_OPENGL_API mult_matrixf(const float* m);
void G2O_OPENGL_API mult_matrixd(const double* m);
void G2O_OPENGL_API scalef(float x, float y, float z);
void G2O_OPENGL_API translatef(float x, float y, float z);
void G2O_OPENGL_API rotatef(float angle, float x, float y, float z);
void G2O_OPENGL_API point_size(float s);
void G2O_OPENGL_API line_width(float w);
void G2O_OPENGL_API matrix_mode(unsigned int mode);
void G2O_OPENGL_API load_identity();
void G2O_OPENGL_API viewport(int x, int y, int w, int h);
void G2O_OPENGL_API clear_color(float r, float g, float b, float a);
void G2O_OPENGL_API clear(unsigned int mask);
void G2O_OPENGL_API push_attrib(unsigned int mask);
// Overloads that accept `Capability` values for convenience.
void G2O_OPENGL_API push_attrib(Capability cap);
void G2O_OPENGL_API push_attrib(std::initializer_list<Capability> caps);
void G2O_OPENGL_API pop_attrib();
void G2O_OPENGL_API color4f(float r, float g, float b, float a);
void G2O_OPENGL_API shade_model(unsigned int mode);
void G2O_OPENGL_API shade_model(Capability mode);
void G2O_OPENGL_API blend_func(unsigned int sfactor, unsigned int dfactor);
void G2O_OPENGL_API blend_func(Capability sfactor, Capability dfactor);
// Simple GLU-like helpers (implemented without libGLU)
void G2O_OPENGL_API glu_perspective(double fovy, double aspect, double zNear,
                                    double zFar);
void G2O_OPENGL_API glu_look_at(double eyex, double eyey, double eyez,
                                double centerx, double centery, double centerz,
                                double upx, double upy, double upz);
bool G2O_OPENGL_API is_enabled(Capability cap);
void G2O_OPENGL_API enable(Capability cap);
void G2O_OPENGL_API disable(Capability cap);
// Generic enable/disable helpers accepting raw GL enums
bool G2O_OPENGL_API is_enabled(unsigned int cap);
void G2O_OPENGL_API enable(unsigned int cap);
void G2O_OPENGL_API disable(unsigned int cap);

// Higher-level helpers (optional fallbacks)
void G2O_OPENGL_API draw_sphere(float radius);
void G2O_OPENGL_API draw_cylinder(float radius, float height);
void G2O_OPENGL_API draw_disk(float radius);

}  // namespace g2o::opengl

#endif  // G2O_OPENGL_INTERFACE_H
