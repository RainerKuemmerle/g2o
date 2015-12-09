#ifndef  __FREEGLUT_EXT_H__
#define  __FREEGLUT_EXT_H__

/*
 * freeglut_ext.h
 *
 * The non-GLUT-compatible extensions to the freeglut library include file
 *
 * Copyright (c) 1999-2000 Pawel W. Olszta. All Rights Reserved.
 * Written by Pawel W. Olszta, <olszta@sourceforge.net>
 * Creation date: Thu Dec 2 1999
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * PAWEL W. OLSZTA BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "g2o/stuff/opengl_wrapper.h"

#ifdef _MSC_VER
// We are using a Microsoft compiler:
#ifdef G2O_SHARED_LIBS
#  ifdef freeglut_minimal_EXPORTS
#    define G2O_FGAPI __declspec(dllexport)
#  else
#    define G2O_FGAPI __declspec(dllimport)
#  endif
#else
#  define G2O_FGAPI
#endif
#else
// Not Microsoft compiler so set empty definition:
#  define G2O_FGAPI
#endif

namespace freeglut_minimal {

  enum FontID {
    GLUT_STROKE_ROMAN,
    GLUT_STROKE_MONO_ROMAN
  };

  /* The stroke font structures */

  typedef struct tagSFG_StrokeVertex SFG_StrokeVertex;
  struct tagSFG_StrokeVertex
  {
    GLfloat         X, Y;
  };

  typedef struct tagSFG_StrokeStrip SFG_StrokeStrip;
  struct tagSFG_StrokeStrip
  {
    int             Number;
    const SFG_StrokeVertex* Vertices;
  };

  typedef struct tagSFG_StrokeChar SFG_StrokeChar;
  struct tagSFG_StrokeChar
  {
    GLfloat         Right;
    int             Number;
    const SFG_StrokeStrip* Strips;
  };

  typedef struct tagSFG_StrokeFont SFG_StrokeFont;
  struct tagSFG_StrokeFont
  {
    char*           Name;                       /* The source font name      */
    int             Quantity;                   /* Number of chars in font   */
    GLfloat         Height;                     /* Height of the characters  */
    const SFG_StrokeChar** Characters;          /* The characters mapping    */
  };

  extern const SFG_StrokeFont fgStrokeRoman;
  extern const SFG_StrokeFont fgStrokeMonoRoman;

  G2O_FGAPI void glutStrokeString(FontID font, const char* string);

  /*
   * Font stuff, see freeglut_font.c
   */
  G2O_FGAPI GLfloat glutStrokeHeight(FontID font);

  /*
   * Return the width of a string drawn using a stroke font
   */
  G2O_FGAPI int glutStrokeLength(FontID fontID, const char* string);

} // end namespace

/*** END OF FILE ***/

#endif /* __FREEGLUT_EXT_H__ */
