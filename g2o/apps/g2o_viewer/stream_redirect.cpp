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

StreamRedirect::StreamRedirect(std::ostream &stream, QPlainTextEdit *te)
    : stream_(stream), te_(te) {
  old_buf_ = stream.rdbuf();
  stream_.rdbuf(this);
}

StreamRedirect::~StreamRedirect() {
  if (!buffer_.empty()) xsputn(buffer_.c_str(), buffer_.size());
  stream_.rdbuf(old_buf_);
}

std::char_traits<char>::int_type StreamRedirect::overflow(int_type v) {
  mutex_.lock();
  if (v == '\n') {
    te_->appendPlainText(QString::fromLatin1(buffer_.c_str(), buffer_.size()));
    buffer_.clear();
  } else
    buffer_.push_back(v);

  mutex_.unlock();
  return v;
}

std::streamsize StreamRedirect::xsputn(const char *p, std::streamsize n) {
  mutex_.lock();
  buffer_.append(p, p + n);

  while (true) {
    std::string::size_type pos = buffer_.find('\n');
    if (pos != std::string::npos) {
      te_->appendPlainText(QString::fromLatin1(buffer_.c_str(), pos));
      buffer_.erase(buffer_.begin(), buffer_.begin() + pos + 1);
    } else
      break;
  }

  mutex_.unlock();
  return n;
}
