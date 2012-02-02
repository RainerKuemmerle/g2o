// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle
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

#ifndef G2O_SCOPED_POINTER_H
#define G2O_SCOPED_POINTER_H

#include <cassert>

namespace g2o {

  namespace {
    struct ScopedPointerDeleter
    {
      template<typename T>
      void operator()(T* t) { delete t;}
    };
  }

  /**
   * \brief a scoped pointer for an objectarray, i.e., object will be deleted on leaving the scope
   */
  template <typename T, typename Del = ScopedPointerDeleter>
  class ScopedPointer 
  {
    public:
      ScopedPointer(T* t = 0) : _pointer(t) {}

      ~ScopedPointer()
      {
        Del deleter;
        deleter(_pointer);
      }

      //! dereference the pointer
      T& operator*() const { return *_pointer;}
      //! access the pointer via ->
      T* operator->() const { return _pointer;}
      //! return the pointer
      T* get() const { return _pointer;}

      /**
       * store another pointer inside
       */
      void reset(T* p)
      {
        assert((p == 0 || p != _pointer) && "ScopedPointer should not reset with itself");
        ScopedPointer aux(p);
        swap(aux);
      }

      /**
       * swap with another pointer
       */
      void swap(ScopedPointer& b)
      {
        T* aux = b._pointer;
        b._pointer = _pointer;
        _pointer = aux;
      }

    protected:
      T* _pointer;

      // do not allow to copy the object
    private:
      ScopedPointer(const ScopedPointer&);
      const ScopedPointer& operator=(const ScopedPointer&);
  };

  /**
   * \brief a scoped pointer for an array, i.e., array will be deleted on leaving the scope
   */
  template <typename T>
  class ScopedArray
  {
    public:
      ScopedArray(T* t = 0) : _pointer(t) {}

      ~ScopedArray()
      {
        delete[] _pointer;
      }

      T & operator[](std::ptrdiff_t i) const
      {
        assert(_pointer != 0 && i >= 0);
        return _pointer[i];
      }

      //! return the pointer
      T* get() const { return _pointer;}

      /**
       * store another array pointer inside
       */
      void reset(T* p)
      {
        assert((p == 0 || p != _pointer) && "ScopedPointer should not reset with itself");
        ScopedArray aux(p);
        swap(aux);
      }

      /**
       * swap with another pointer
       */
      void swap(ScopedArray & b)
      {
        T* aux = b._pointer;
        b._pointer = _pointer;
        _pointer = aux;
      }

    protected:
      T* _pointer;

      // do not allow to copy the object
    private:
      ScopedArray(const ScopedArray&);
      const ScopedArray& operator=(const ScopedArray&);
  };

} // end namespace

#endif
