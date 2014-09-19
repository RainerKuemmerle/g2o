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

#include "edge_se2_segment2d_line.h"

#ifdef WINDOWS
#include <windows.h>
#endif

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

namespace g2o {

  EdgeSE2Segment2DLine::EdgeSE2Segment2DLine() :
    BaseBinaryEdge<2, Vector2D, VertexSE2, VertexSegment2D>()
  {
  }

  bool EdgeSE2Segment2DLine::read(std::istream& is)
  {
    for (size_t i = 0; i < 2 ; i++)
      is >> _measurement[i];
    for (size_t i = 0; i < 2 ; i++)
      for (size_t j = i; j < 2 ; j++) {
        is >> _information (i,j);
        _information (j,i) = _information (i,j);
      }
    return true;
  }

  bool EdgeSE2Segment2DLine::write(std::ostream& os) const
  {
    for (size_t i = 0; i < 2 ; i++)
      os << _measurement[i] << " ";
    for (size_t i = 0; i < 2 ; i++)
      for (size_t j = i; j < 2 ; j++) {
        os << _information (i,j) << " ";
      }
    return os.good();
  }

} // end namespace
