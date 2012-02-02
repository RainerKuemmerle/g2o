/*
 * Copyright 2005, 2006, 2007
 * Nick Galbreath -- nickg [at] modp [dot] com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the modp.com nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This is the standard "new" BSD license:
 * http://www.opensource.org/licenses/bsd-license.php
 */

#ifndef G2O_FAST_OUTPUT_H
#define G2O_FAST_OUTPUT_H

#include <cstdio>
#include <stdint.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

inline void strreverse(char* begin, char* end)
{
  char aux;
  while (end > begin)
    aux = *end, *end-- = *begin, *begin++ = aux;
}

inline int modp_dtoa(double value, char* str, int prec)
{
  static const double pow10[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

  /* Hacky test for NaN
   * under -fast-math this won't work, but then you also won't
   * have correct nan values anyways.  The alternative is
   * to link with libmath (bad) or hack IEEE double bits (bad)
   */
  if (! (value == value)) {
    str[0] = 'n'; str[1] = 'a'; str[2] = 'n'; str[3] = '\0';
    assert(0);
    return 3;
  }
  /* if input is larger than thres_max, revert to exponential */
  const double thres_max = (double)(0x7FFFFFFF);

  double diff = 0.0;
  char* wstr = str;

  if (prec < 0) {
    prec = 0;
  } else if (prec > 9) {
    /* precision of >= 10 can lead to overflow errors */
    prec = 9;
  }


  /* we'll work in positive values and deal with the
     negative sign issue later */
  int neg = 0;
  if (value < 0) {
    neg = 1;
    value = -value;
  }


  int whole = (int) value;
  double tmp = (value - whole) * pow10[prec];
  uint32_t frac = (uint32_t)(tmp);
  diff = tmp - frac;

  if (diff > 0.5) {
    ++frac;
    /* handle rollover, e.g.  case 0.99 with prec 1 is 1.0  */
    if (frac >= pow10[prec]) {
      frac = 0;
      ++whole;
    }
  } else if (diff == 0.5 && ((frac == 0) || (frac & 1))) {
    /* if halfway, round up if odd, OR
       if last digit is 0.  That last part is strange */
    ++frac;
  }

  /* for very large numbers switch back to native sprintf for exponentials.
     anyone want to write code to replace this? */
  /*
     normal printf behavior is to print EVERY whole number digit
     which can be 100s of characters overflowing your buffers == bad
     */
  if (value > thres_max) {
    return sprintf(str, "%e", neg ? -value : value);
  }

  if (prec == 0) {
    diff = value - whole;
    if (diff > 0.5) {
      /* greater than 0.5, round up, e.g. 1.6 -> 2 */
      ++whole;
    } else if (diff == 0.5 && (whole & 1)) {
      /* exactly 0.5 and ODD, then round up */
      /* 1.5 -> 2, but 2.5 -> 2 */
      ++whole;
    }
  } else {
    int count = prec;
    // now do fractional part, as an unsigned number
    do {
      --count;
      *wstr++ = (char)(48 + (frac % 10));
    } while (frac /= 10);
    // add extra 0s
    while (count-- > 0) *wstr++ = '0';
    // add decimal
    *wstr++ = '.';
  }

  // do whole part
  // Take care of sign
  // Conversion. Number is reversed.
  do *wstr++ = (char)(48 + (whole % 10)); while (whole /= 10);
  if (neg) {
    *wstr++ = '-';
  }
  //*wstr='\0';
  strreverse(str, wstr-1);
  return wstr - str;
}

inline int modp_uitoa10(uint32_t value, char* str)
{
  char* wstr=str;
  // Conversion. Number is reversed.
  do *wstr++ = (char)(48 + (value % 10)); while (value /= 10);
  //*wstr='\0';
  // Reverse string
  strreverse(str, wstr-1);
  return wstr - str;
}

inline int modp_itoa10(int32_t value, char* str)
{
  char* wstr=str;
  // Take care of sign
  unsigned int uvalue = (value < 0) ? -value : value;
  // Conversion. Number is reversed.
  do *wstr++ = (char)(48 + (uvalue % 10)); while(uvalue /= 10);
  if (value < 0) *wstr++ = '-';
  *wstr='\0';

  // Reverse string
  strreverse(str,wstr-1);
  return wstr - str;
}

#ifdef __cplusplus
}
#endif

#endif
