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

#include "os_specific.h"

#ifdef WINDOWS

int vasprintf(char** strp, const char* fmt, va_list ap)
{
  int n;
  int size = 100;
  char* p;
  char* np;

  if ((p = (char*)malloc(size * sizeof(char))) == NULL)
    return -1;

  while (1) {
#ifdef _MSC_VER
    n = vsnprintf_s(p, size, size - 1, fmt, ap);
#else
    n = vsnprintf(p, size, fmt, ap);
#endif
    if (n > -1 && n < size) {
      *strp = p;
      return n;
    }
    if (n > -1)
      size = n+1;
    else
      size *= 2;
    if ((np = (char*)realloc (p, size * sizeof(char))) == NULL) {
      free(p);
      return -1;
    } else
      p = np;
  }
}


#endif
