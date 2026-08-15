#ifndef G2O_OPENGL_API_H
#define G2O_OPENGL_API_H

// Cross-platform export macro for g2o OpenGL helpers
#ifdef _MSC_VER
#ifdef G2O_SHARED_LIBS
#ifdef opengl_helper_EXPORTS
#define G2O_OPENGL_API __declspec(dllexport)
#else
#define G2O_OPENGL_API __declspec(dllimport)
#endif
#else
#define G2O_OPENGL_API
#endif
#else
#define G2O_OPENGL_API
#endif

#endif  // G2O_OPENGL_API_H
