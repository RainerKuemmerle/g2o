#ifndef SLAM_DRIVER_H
#define SLAM_DRIVER_H

#include <string>
#include <vector>

namespace SlamParser {

class SlamContext;

class Driver
{
public:
    /// construct a new parser driver context
    Driver(class SlamContext& slamContext_);

    /// enable debug output in the flex scanner
    bool trace_scanning;

    /// enable debug output in the bison parser
    bool trace_parsing;

    /// stream name (file or input stream) used for error messages.
    std::string streamname;

    /** Invoke the scanner and parser for a stream.
     * @param in  input stream
     * @param sname  stream name for error messages
     * @return    true if successfully parsed
     */
    bool parse_stream(std::istream& in,
          const std::string& sname = "stream input");

    /** Invoke the scanner and parser on an input string.
     * @param input  input string
     * @param sname  stream name for error messages
     * @return    true if successfully parsed
     */
    bool parse_string(const std::string& input,
          const std::string& sname = "string stream");

    /** Invoke the scanner and parser on a file. Use parse_stream with a
     * std::ifstream if detection of file reading errors is required.
     * @param filename  input file name
     * @return    true if successfully parsed
     */
    bool parse_file(const std::string& filename);

    // To demonstrate pure handling of parse errors, instead of
    // simply dumping them on the standard error output, we will pass
    // them to the driver using the following two member functions.

    /** Error handling with associated line number. This can be modified to
     * output the error e.g. to a dialog box. */
    void error(const class location& l, const std::string& m);

    /** General error handling. This can be modified to output the error
     * e.g. to a dialog box. */
    void error(const std::string& m);

    /** Pointer to the current lexer instance, this is used to connect the
     * parser to the scanner. It is used in the yylex macro. */
    class Scanner* lexer;

    /** Reference to the calculator context filled during parsing of the
     * expressions. */
    class SlamContext& slamContext;
};

}

#endif
