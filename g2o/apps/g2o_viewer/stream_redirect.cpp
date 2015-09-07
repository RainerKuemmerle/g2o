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

  std::string::size_type pos = 0;
  while (1) {
    pos = _buffer.find('\n');
    if (pos != std::string::npos) {
      _te->appendPlainText(QString::fromLatin1(_buffer.c_str(), pos));
      _buffer.erase(_buffer.begin(), _buffer.begin() + pos + 1);
    } else
      break;
  }

  _mutex.unlock();
  return n;
}
