// Qt OpenGL backend: use Qt's OpenGL bindings when available
#include "opengl_interface.h"

#include <QOpenGLContext>
#include <QOpenGLFunctions_1_1>
#include <cmath>
namespace g2o::opengl {

namespace {
// Helper to get Qt's legacy (1.1) GL function wrappers from the current
// context.
inline QOpenGLFunctions_1_1* glf() {
  QOpenGLContext* ctx = QOpenGLContext::currentContext();
  if (!ctx) return nullptr;
  static thread_local QOpenGLFunctions_1_1 funcs;
  static thread_local QOpenGLContext* lastCtx = nullptr;
  if (lastCtx != ctx) {
    funcs.initializeOpenGLFunctions();
    lastCtx = ctx;
  }
  return &funcs;
}

// Mapping from our Capability enum to OpenGL constants
GLenum to_gl_enum(Capability cap) {
  switch (cap) {
    case Capability::NORMALIZE:
      return GL_NORMALIZE;
    case Capability::LIGHTING:
      return GL_LIGHTING;
    case Capability::BLEND:
      return GL_BLEND;
    case Capability::DEPTH_TEST:
      return GL_DEPTH_TEST;
    case Capability::LINE_SMOOTH:
      return GL_LINE_SMOOTH;
    case Capability::ENABLE_BIT:
      return GL_ENABLE_BIT;
    case Capability::POINT_BIT:
      return GL_POINT_BIT;
    case Capability::COLOR_BUFFER_BIT:
      return GL_COLOR_BUFFER_BIT;
    case Capability::DEPTH_BUFFER_BIT:
      return GL_DEPTH_BUFFER_BIT;
    case Capability::FLAT:
      return GL_FLAT;
    case Capability::SMOOTH:
      return GL_SMOOTH;
    case Capability::SRC_ALPHA:
      return GL_SRC_ALPHA;
    case Capability::ONE_MINUS_SRC_ALPHA:
      return GL_ONE_MINUS_SRC_ALPHA;
    default:
      return static_cast<GLenum>(cap);
  }
}
}  // namespace

void init() { /* nothing */ }
void begin_triangles() {
  if (auto* f = glf()) f->glBegin(GL_TRIANGLES);
}
void begin_quads() {
  if (auto* f = glf()) f->glBegin(GL_QUADS);
}
void begin_lines() {
  if (auto* f = glf()) f->glBegin(GL_LINES);
}
void begin_line_strip() {
  if (auto* f = glf()) f->glBegin(GL_LINE_STRIP);
}
void begin_points() {
  if (auto* f = glf()) f->glBegin(GL_POINTS);
}
void end() {
  if (auto* f = glf()) f->glEnd();
}
void vertex3f(float x, float y, float z) {
  if (auto* f = glf()) f->glVertex3f(x, y, z);
}
void vertex2f(float x, float y) {
  if (auto* f = glf()) f->glVertex2f(x, y);
}
void color3f(float r, float g, float b) {
  if (auto* f = glf()) f->glColor3f(r, g, b);
}
void normal3f(float x, float y, float z) {
  if (auto* f = glf()) f->glNormal3f(x, y, z);
}
void push_matrix() {
  if (auto* f = glf()) f->glPushMatrix();
}
void pop_matrix() {
  if (auto* f = glf()) f->glPopMatrix();
}
void mult_matrixf(const float* m) {
  if (auto* f = glf()) f->glMultMatrixf(m);
}
void scalef(float x, float y, float z) {
  if (auto* f = glf()) f->glScalef(x, y, z);
}
void translatef(float x, float y, float z) {
  if (auto* f = glf()) f->glTranslatef(x, y, z);
}
void rotatef(float angle, float x, float y, float z) {
  if (auto* f = glf()) f->glRotatef(angle, x, y, z);
}
void point_size(float s) {
  if (auto* f = glf()) f->glPointSize(s);
}
void line_width(float w) {
  if (auto* f = glf()) f->glLineWidth(w);
}
bool is_enabled(Capability cap) {
  if (auto* f = glf()) return f->glIsEnabled(to_gl_enum(cap)) == GL_TRUE;
  return false;
}
void enable(Capability cap) {
  if (auto* f = glf()) f->glEnable(to_gl_enum(cap));
}
void disable(Capability cap) {
  if (auto* f = glf()) f->glDisable(to_gl_enum(cap));
}

void mult_matrixd(const double* m) {
  if (auto* f = glf()) f->glMultMatrixd(m);
}
void matrix_mode(unsigned int mode) {
  if (auto* f = glf()) f->glMatrixMode(static_cast<GLenum>(mode));
}
void load_identity() {
  if (auto* f = glf()) f->glLoadIdentity();
}
void viewport(int x, int y, int w, int h) {
  if (auto* f = glf()) f->glViewport(x, y, w, h);
}
void clear_color(float r, float g, float b, float a) {
  if (auto* f = glf()) f->glClearColor(r, g, b, a);
}
void clear(unsigned int mask) {
  if (auto* f = glf()) f->glClear(static_cast<GLbitfield>(mask));
}
void push_attrib(unsigned int mask) {
  if (auto* f = glf()) f->glPushAttrib(static_cast<GLbitfield>(mask));
}
void pop_attrib() {
  if (auto* f = glf()) f->glPopAttrib();
}

void push_attrib(Capability cap) {
  if (auto* f = glf())
    f->glPushAttrib(static_cast<GLbitfield>(to_gl_enum(cap)));
}

void push_attrib(std::initializer_list<Capability> caps) {
  unsigned int mask = 0;
  for (auto c : caps) mask |= to_gl_enum(c);
  if (auto* f = glf()) f->glPushAttrib(static_cast<GLbitfield>(mask));
}
void color4f(float r, float g, float b, float a) {
  if (auto* f = glf()) f->glColor4f(r, g, b, a);
}
void shade_model(unsigned int mode) {
  if (auto* f = glf()) f->glShadeModel(static_cast<GLenum>(mode));
}
void blend_func(unsigned int sfactor, unsigned int dfactor) {
  if (auto* f = glf())
    f->glBlendFunc(static_cast<GLenum>(sfactor), static_cast<GLenum>(dfactor));
}

// Overloads that accept Capability for convenience
void shade_model(Capability mode) { glShadeModel(to_gl_enum(mode)); }
void blend_func(Capability sfactor, Capability dfactor) {
  if (auto* f = glf()) f->glBlendFunc(to_gl_enum(sfactor), to_gl_enum(dfactor));
}

void glu_perspective(double fovy, double aspect, double zNear, double zFar) {
  // Implement in terms of glFrustum to avoid libGLU dependency
  const double deg2rad = M_PI / 180.0;
  double f = fovy / 2.0;
  double top = zNear * tan(f * deg2rad);
  double bottom = -top;
  double right = top * aspect;
  double left = -right;
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glFrustum(left, right, bottom, top, zNear, zFar);
  glMatrixMode(GL_MODELVIEW);
}

void glu_look_at(double eyex, double eyey, double eyez, double centerx,
                 double centery, double centerz, double upx, double upy,
                 double upz) {
  // Compute forward vector
  double fx = centerx - eyex;
  double fy = centery - eyey;
  double fz = centerz - eyez;
  // normalize f
  double rlf = 1.0 / sqrt((fx * fx) + (fy * fy) + (fz * fz));
  fx *= rlf;
  fy *= rlf;
  fz *= rlf;
  // compute up normalized
  double upn_x = upx;
  double upn_y = upy;
  double upn_z = upz;
  double rlu = 1.0 / sqrt((upn_x * upn_x) + (upn_y * upn_y) + (upn_z * upn_z));
  upn_x *= rlu;
  upn_y *= rlu;
  upn_z *= rlu;
  // side = f x up
  double sx = (fy * upn_z) - (fz * upn_y);
  double sy = (fz * upn_x) - (fx * upn_z);
  double sz = (fx * upn_y) - (fy * upn_x);
  double rls = 1.0 / sqrt((sx * sx) + (sy * sy) + (sz * sz));
  sx *= rls;
  sy *= rls;
  sz *= rls;
  // recompute up = s x f
  double ux = (sy * fz) - (sz * fy);
  double uy = (sz * fx) - (sx * fz);
  double uz = (sx * fy) - (sy * fx);

  double m[16] = {sx, ux, -fx, 0.0, sy,  uy,  -fy, 0.0,
                  sz, uz, -fz, 0.0, 0.0, 0.0, 0.0, 1.0};

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glMultMatrixd(m);
  glTranslated(-eyex, -eyey, -eyez);
}

bool is_enabled(unsigned int cap) {
  return glIsEnabled(static_cast<GLenum>(cap)) == GL_TRUE;
}
void enable(unsigned int cap) { glEnable(static_cast<GLenum>(cap)); }
void disable(unsigned int cap) { glDisable(static_cast<GLenum>(cap)); }

// Basic draw helpers implemented with immediate-mode fallbacks
void draw_sphere(float radius) {
  // naive lat-long tessellation (low-res) to avoid GLU dependency
  const int lats = 16;
  const int longs = 16;
  for (int i = 0; i <= lats; ++i) {
    float lat0 = M_PI * (-0.5F + static_cast<float>(i - 1) / lats);
    float z0 = sinf(lat0);
    float zr0 = cosf(lat0);

    float lat1 = M_PI * (-0.5F + static_cast<float>(i) / lats);
    float z1 = sinf(lat1);
    float zr1 = cosf(lat1);

    begin_triangles();
    for (int j = 0; j <= longs; ++j) {
      float lng = 2 * M_PI * static_cast<float>(j - 1) / longs;
      float x = cosf(lng);
      float y = sinf(lng);
      vertex3f(x * zr0 * radius, y * zr0 * radius, z0 * radius);
      vertex3f(x * zr1 * radius, y * zr1 * radius, z1 * radius);
    }
    end();
  }
}

void draw_cylinder(float radius, float height) {
  // simple cylinder using triangle strip
  const int slices = 24;
  begin_quads();
  for (int i = 0; i < slices; ++i) {
    float theta = (2.0F * M_PI * i) / slices;
    float nx = cosf(theta);
    float ny = sinf(theta);
    vertex3f(nx * radius, ny * radius, -height / 2);
    vertex3f(nx * radius, ny * radius, height / 2);
  }
  end();
}

void draw_disk(float radius) {
  const int slices = 24;
  begin_triangles();
  for (int i = 0; i < slices; ++i) {
    float a0 = (2.0F * M_PI * i) / slices;
    float a1 = (2.0F * M_PI * (i + 1)) / slices;
    vertex3f(0.0F, 0.0F, 0.0F);
    vertex3f(cosf(a0) * radius, sinf(a0) * radius, 0.0F);
    vertex3f(cosf(a1) * radius, sinf(a1) * radius, 0.0F);
  }
  end();
}

}  // namespace g2o::opengl
