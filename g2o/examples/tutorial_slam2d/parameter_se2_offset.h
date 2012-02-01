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

#ifndef G2O_TUTORIAL_PARAMETER_SE2_OFFSET_H
#define G2O_TUTORIAL_PARAMETER_SE2_OFFSET_H

#include "g2o/core/cache.h"
#include "se2.h"
#include "g2o_tutorial_slam2d_api.h"

namespace g2o {
  namespace tutorial {

    class G2O_TUTORIAL_SLAM2D_API ParameterSE2Offset: public Parameter
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        ParameterSE2Offset();

        void setOffset(const SE2& offset = SE2());

        const SE2& offset() const { return _offset;}
        const SE2& inverseOffset() const { return _inverseOffset;}

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

      protected:
        SE2 _offset;
        SE2 _inverseOffset;
    };

    class G2O_TUTORIAL_SLAM2D_API CacheSE2Offset: public Cache
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        const SE2& w2n() const { return _w2n;}
        const SE2& n2w() const { return _n2w;}

      protected:
        virtual void updateImpl();
        virtual bool resolveDependancies();

        ParameterSE2Offset* _offsetParam;
        SE2 _w2n, _n2w; 
    };

  } // end namespace tutorial
} // end namespace

#endif
