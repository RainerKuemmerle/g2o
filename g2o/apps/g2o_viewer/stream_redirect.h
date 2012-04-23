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
