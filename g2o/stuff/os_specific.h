// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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

#ifndef G2O_OS_SPECIFIC_HH_
#define G2O_OS_SPECIFIC_HH_

#ifdef WINDOWS
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#ifndef _WINDOWS
#include <sys/time.h>
#endif
#define drand48() ((double) rand()/(double)RAND_MAX)

#ifdef __cplusplus
extern "C" {
#endif

int vasprintf(char** strp, const char* fmt, va_list ap);

#ifdef __cplusplus
}
#endif

#endif

#ifdef UNIX
#include <sys/time.h>
// nothing to do on real operating systems
#endif

#endif
