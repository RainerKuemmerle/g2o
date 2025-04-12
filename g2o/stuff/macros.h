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

#ifndef G2O_MACROS_H
#define G2O_MACROS_H

#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * 0.01745329251994329575)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 57.29577951308232087721)
#endif

// GCC the one and only
#if defined(__GNUC__)
#define G2O_ATTRIBUTE_CONSTRUCTOR(func)                \
  static void func(void) __attribute__((constructor)); \
  static void func(void)

#define G2O_ATTRIBUTE_UNUSED __attribute__((unused))
#define G2O_ATTRIBUTE_WARNING(func) func __attribute__((warning))
#define G2O_ATTRIBUTE_DEPRECATED(func) func __attribute__((deprecated))

// MSVC on Windows
#elif defined _MSC_VER

/**
Modified by Mark Pupilli from:

        "Initializer/finalizer sample for MSVC and GCC.
    2010 Joe Lowe. Released into the public domain."

        "For MSVC, places a ptr to the function in the user initializer section
(.CRT$XCU), basically the same thing the compiler does for the constructor calls
for static C++ objects. For GCC, uses a constructor attribute."

        (As posted on Stack OVerflow)
*/
#define G2O_ATTRIBUTE_CONSTRUCTOR(f)                               \
  __pragma(section(".CRT$XCU", read)) static void __cdecl f(void); \
  __declspec(allocate(".CRT$XCU")) void(__cdecl * f##_)(void) = f; \
  static void __cdecl f(void)

#define G2O_ATTRIBUTE_UNUSED
#define G2O_ATTRIBUTE_WARNING(func) func
#define G2O_ATTRIBUTE_DEPRECATED(func) func

#include <float.h>

// unknown compiler
#else
#define G2O_ATTRIBUTE_CONSTRUCTOR(func) func
#define G2O_ATTRIBUTE_UNUSED
#define G2O_ATTRIBUTE_WARNING(func) func
#define G2O_ATTRIBUTE_DEPRECATED(func) func

#endif

// some macros that are only useful for c++
#ifdef __cplusplus

#ifndef PVAR
#define PVAR(s) #s << " = " << (s) << std::flush
#endif

#ifndef FIXED
#define FIXED(s) std::fixed << s << std::resetiosflags(std::ios_base::fixed)
#endif

#endif  // __cplusplus

#endif
