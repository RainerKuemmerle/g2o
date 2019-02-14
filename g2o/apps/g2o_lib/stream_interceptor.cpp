#include "stream_interceptor.h"

StreamInterceptor::StreamInterceptor(std::ostream &ostream, CallbackFunc callbackfunc)
    : m_ostream(ostream), m_callbackfunc(callbackfunc), m_lineBuffer("") {
  m_tempstreambuf = m_ostream.rdbuf(this);
}

StreamInterceptor::~StreamInterceptor() {
  m_ostream.rdbuf(m_tempstreambuf);
}

streamsize StreamInterceptor::xsputn(const char *msg, streamsize count) {
  processCharacters(msg, count);
  return count;
}

int StreamInterceptor::overflow(int_type c) {
  if (c == 10) {
    flushLineBuffer();
  } else {
    if (c != EOF) {
      char z = c;
      processCharacters(&z, 1);
    }
  }
  return c;
}

void StreamInterceptor::flush() {
  if (!m_lineBuffer.empty()) {
    flushLineBuffer();
  }
}

void StreamInterceptor::processCharacters(const char *msg, streamsize count) {
  m_lineBuffer += string(msg, (int) count);
}

void StreamInterceptor::flushLineBuffer() {
  m_callbackfunc(m_lineBuffer.c_str());
  m_lineBuffer = "";
}
