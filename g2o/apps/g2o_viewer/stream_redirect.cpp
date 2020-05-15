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

#include "stream_redirect.h"

#include <QPlainTextEdit>
#include <QString>

#include <iostream>
using namespace std;

StreamRedirect::StreamRedirect(std::ostream &stream, QPlainTextEdit* te):
  _stream(stream), _te(te)
{
  _old_buf = stream.rdbuf();
  _stream.rdbuf(this);
}

StreamRedirect::~StreamRedirect()
{
  if (!_buffer.empty())
    xsputn(_buffer.c_str(), _buffer.size());
  _stream.rdbuf(_old_buf);
}

std::char_traits<char>::int_type StreamRedirect::overflow(int_type v)
{
  _mutex.lock();
  if (v == '\n') {
    _te->appendPlainText(QString::fromLatin1(_buffer.c_str(), _buffer.size()));
    _buffer.clear();
  }
  else
    _buffer.push_back(v);

  _mutex.unlock();
  return v;
}

std::streamsize StreamRedirect::xsputn(const char *p, std::streamsize n)
{
  _mutex.lock();
  _buffer.append(p, p + n);

  while (1) {
    std::string::size_type pos = _buffer.find('\n');
    if (pos != std::string::npos) {
      _te->appendPlainText(QString::fromLatin1(_buffer.c_str(), pos));
      _buffer.erase(_buffer.begin(), _buffer.begin() + pos + 1);
    } else
      break;
  }

  _mutex.unlock();
  return n;
}
