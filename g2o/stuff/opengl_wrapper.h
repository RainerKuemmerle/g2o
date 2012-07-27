#ifndef G2O_OPENGL_WRAPPER_H
#define G2O_OPENGL_WRAPPER_H

#include "g2o/config.h"

#ifdef WINDOWS
#include <windows.h>
#endif

#ifdef G2O_OPENGL_FOUND
# ifdef __APPLE__
#  include <OpenGL/gl.h>
# else
#  include <GL/gl.h>
# endif
#endif

#endif
