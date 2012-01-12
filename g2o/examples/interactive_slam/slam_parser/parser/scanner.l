%{ /*** C/C++ Declarations ***/

#include <vector>
#include <string>

#include "scanner.h"

/* import the parser's token type into a local typedef */
typedef SlamParser::Parser::token token;
typedef SlamParser::Parser::token_type token_type;

/* By default yylex returns int, we use token_type. Unfortunately yyterminate
 * by default returns 0, which is not of token_type. */
#define yyterminate() return token::END

/* This disables inclusion of unistd.h, which is not available under Visual C++
 * on Win32. The C++ scanner uses STL streams instead. */
#define YY_NO_UNISTD_H

%}

/*** Flex Declarations and Options ***/

/* enable c++ scanner class generation */
%option c++

/* change the name of the scanner class. results in "SlamFlexLexer" */
%option prefix="Slam"

/* the manual says "somewhat more optimized" */
%option batch

/* enable scanner to generate debug output. disable this for release
 * versions. */
/*%option debug*/

/* no support for include files is planned */
%option yywrap nounput 

/* enables the use of start condition stacks */
%option stack

/* The following paragraph suffices to track locations accurately. Each time
 * yylex is invoked, the begin position is moved onto the end position. */
%{
#define YY_USER_ACTION  yylloc->columns(yyleng);
%}

%% /*** Regular Expressions Part ***/

 /* code to place at the beginning of yylex() */
%{
    // reset location
    yylloc->step();
%}

 /*** BEGIN EXAMPLE - Change the example lexer rules below ***/

("+"|"-")?[0-9]+ {
  yylval->integerVal = atoi(yytext);
  return token::INTEGER;
}

(\+|-)?([0-9]+\.?[0-9]*|\.[0-9]+)([eE](\+|-)?[0-9]+)? {
  yylval->doubleVal = atof(yytext);
  return token::DOUBLE;
}

"ADD" {
  /*yylval->stringVal = new std::string(yytext, yyleng);*/
  return token::ADD;
}

"VERTEX_XYT" {
  yylval->stringVal = new std::string(yytext, yyleng);
  return token::V_SE2;
}

"VERTEX_XYZRPY" {
  yylval->stringVal = new std::string(yytext, yyleng);
  return token::V_SE3;
}

"EDGE_XYT" {
  yylval->stringVal = new std::string(yytext, yyleng);
  return token::E_SE2;
}

"EDGE_XYZRPY" {
  yylval->stringVal = new std::string(yytext, yyleng);
  return token::E_SE3;
}

"FIX" {
  /*yylval->stringVal = new std::string(yytext, yyleng);*/
  return token::FIX;
}

"SOLVE_STATE" {
  /*yylval->stringVal = new std::string(yytext, yyleng);*/
  return token::SOLVE_STATE;
}

"QUERY_STATE" {
  /*yylval->stringVal = new std::string(yytext, yyleng);*/
  return token::QUERY_STATE;
}

[A-Za-z][A-Za-z0-9_,.-]* {
    yylval->stringVal = new std::string(yytext, yyleng);
    return token::STRING;
}

 /* gobble up white-spaces */
[ \t\r]+ {
    yylloc->step();
}

 /* gobble up end-of-lines */
\n {
    yylloc->lines(yyleng); yylloc->step();
    return token::EOL;
}

 /* pass all other characters up to bison */
. {
    return static_cast<token_type>(*yytext);
}

 /*** END EXAMPLE - Change the example lexer rules above ***/

%% /*** Additional Code ***/

namespace SlamParser {

Scanner::Scanner(std::istream* in,
		 std::ostream* out)
    : SlamFlexLexer(in, out)
{
}

Scanner::~Scanner()
{
}

void Scanner::set_debug(bool b)
{
    yy_flex_debug = b;
}

}

#ifdef yylex
#undef yylex
#endif

int SlamFlexLexer::yylex()
{
    std::cerr << "in SlamFlexLexer::yylex() !" << std::endl;
    return 0;
}

/* When the scanner receives an end-of-file indication from YY_INPUT, it then
 * checks the yywrap() function. If yywrap() returns false (zero), then it is
 * assumed that the function has gone ahead and set up `yyin' to point to
 * another input file, and scanning continues. If it returns true (non-zero),
 * then the scanner terminates, returning 0 to its caller. */

int SlamFlexLexer::yywrap()
{
    //std::cerr << "in SlamFlexLexer::yywrap() !" << std::endl;
    return 1;
}
