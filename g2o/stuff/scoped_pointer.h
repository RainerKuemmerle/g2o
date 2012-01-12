// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
