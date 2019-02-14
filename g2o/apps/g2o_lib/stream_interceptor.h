#ifndef G2O_STREAM_INTERCEPTOR_H
#define G2O_STREAM_INTERCEPTOR_H

#include <iostream>

using namespace std;

class StreamInterceptor : public streambuf {
public:
    typedef void(*CallbackFunc)(const char *msg);

    StreamInterceptor(ostream& ostream, CallbackFunc callbackFunc);
    virtual ~StreamInterceptor();

protected:
    virtual int_type overflow(int_type c);
    virtual streamsize xsputn(const char *msg, streamsize count);
    void flush();

private:
    void processCharacters(const char *msg, streamsize count);
    void flushLineBuffer();

    ostream& m_ostream;
    streambuf* m_tempstreambuf;
    CallbackFunc m_callbackfunc;
    string m_lineBuffer;
};

#endif //G2O_STREAM_INTERCEPTOR_H
