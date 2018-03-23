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

#ifndef G2O_STREAM_REDIRECT_H
#define G2O_STREAM_REDIRECT_H

#include <iostream>
#include <streambuf>
#include <string>
#include <QMutex>

#include "g2o_viewer_api.h"

class QPlainTextEdit;

/**
 * \brief redirect a stream to a QPlainTextEdit
 */
class G2O_VIEWER_API StreamRedirect : public std::basic_streambuf<char>
{
  public:
    typedef std::char_traits<char>::int_type int_type;

  public:
    StreamRedirect(std::ostream &stream, QPlainTextEdit* te);
    ~StreamRedirect();

  protected:
    virtual std::char_traits<char>::int_type overflow(int_type v);
    virtual std::streamsize xsputn(const char *p, std::streamsize n); 

  private:
    std::ostream& _stream;
    std::streambuf* _old_buf;
    std::string _buffer;
    QPlainTextEdit* _te;
    QMutex _mutex;
};

#endif
