// A Bison parser, made by GNU Bison 3.7.6.

// Skeleton implementation for Bison LALR(1) parsers in C++

// Copyright (C) 2002-2015, 2018-2021 Free Software Foundation, Inc.

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// As a special exception, you may create a larger work that contains
// part or all of the Bison parser skeleton and distribute that work
// under terms of your choice, so long as that work isn't itself a
// parser generator using the skeleton or a modified version thereof
// as a parser skeleton.  Alternatively, if you modify or redistribute
// the parser skeleton itself, you may (at your option) remove this
// special exception, which will cause the skeleton and the resulting
// Bison output files to be licensed under the GNU General Public
// License without this special exception.

// This special exception was added by the Free Software Foundation in
// version 2.2 of Bison.

// DO NOT RELY ON FEATURES THAT ARE NOT DOCUMENTED in the manual,
// especially those whose name start with YY_ or yy_.  They are
// private implementation details that can be changed or removed.

// Take the name prefix into account.
#define yylex slam_parserlex

// First part of user prologue.
#line 1 "parser.yy"
/*** C/C++ Declarations ***/

#include <stdio.h>

#include <string>
#include <vector>

#include "commands.h"

#line 53 "bison_parser.cpp"

#include "bison_parser.h"

// Second part of user prologue.
#line 85 "parser.yy"

#include "driver.h"
#include "scanner.h"
#include "slam_context.h"

/* this "connects" the bison parser in the driver to the flex scanner class
 * object. it defines the yylex() function call to pull the next token from the
 * current lexer object of the driver context. */
#undef yylex
#define yylex driver.lexer->lex

#line 73 "bison_parser.cpp"

#ifndef YY_
#if defined YYENABLE_NLS && YYENABLE_NLS
#if ENABLE_NLS
#include <libintl.h>  // FIXME: INFRINGES ON USER NAME SPACE.
#define YY_(msgid) dgettext("bison-runtime", msgid)
#endif
#endif
#ifndef YY_
#define YY_(msgid) msgid
#endif
#endif

// Whether we are compiled with exception support.
#ifndef YY_EXCEPTIONS
#if defined __GNUC__ && !defined __EXCEPTIONS
#define YY_EXCEPTIONS 0
#else
#define YY_EXCEPTIONS 1
#endif
#endif

#define YYRHSLOC(Rhs, K) ((Rhs)[K].location)
/* YYLLOC_DEFAULT -- Set CURRENT to span from RHS[1] to RHS[N].
   If N is 0, then set CURRENT to the empty location which ends
   the previous symbol: RHS[0] (always defined).  */

#ifndef YYLLOC_DEFAULT
#define YYLLOC_DEFAULT(Current, Rhs, N)                       \
  do                                                          \
    if (N) {                                                  \
      (Current).begin = YYRHSLOC(Rhs, 1).begin;               \
      (Current).end = YYRHSLOC(Rhs, N).end;                   \
    } else {                                                  \
      (Current).begin = (Current).end = YYRHSLOC(Rhs, 0).end; \
    }                                                         \
  while (false)
#endif

// Enable debugging if requested.
#if SLAM_PARSERDEBUG

// A pseudo ostream that takes yydebug_ into account.
#define YYCDEBUG \
  if (yydebug_) (*yycdebug_)

#define YY_SYMBOL_PRINT(Title, Symbol) \
  do {                                 \
    if (yydebug_) {                    \
      *yycdebug_ << Title << ' ';      \
      yy_print_(*yycdebug_, Symbol);   \
      *yycdebug_ << '\n';              \
    }                                  \
  } while (false)

#define YY_REDUCE_PRINT(Rule)             \
  do {                                    \
    if (yydebug_) yy_reduce_print_(Rule); \
  } while (false)

#define YY_STACK_PRINT()             \
  do {                               \
    if (yydebug_) yy_stack_print_(); \
  } while (false)

#else  // !SLAM_PARSERDEBUG

#define YYCDEBUG \
  if (false) std::cerr
#define YY_SYMBOL_PRINT(Title, Symbol) YY_USE(Symbol)
#define YY_REDUCE_PRINT(Rule) static_cast<void>(0)
#define YY_STACK_PRINT() static_cast<void>(0)

#endif  // !SLAM_PARSERDEBUG

#define yyerrok (yyerrstatus_ = 0)
#define yyclearin (yyla.clear())

#define YYACCEPT goto yyacceptlab
#define YYABORT goto yyabortlab
#define YYERROR goto yyerrorlab
#define YYRECOVERING() (!!yyerrstatus_)

namespace slam_parser {
#line 166 "bison_parser.cpp"

/// Build a parser object.
Parser::Parser(class Driver& driver_yyarg)
#if SLAM_PARSERDEBUG
    : yydebug_(false),
      yycdebug_(&std::cerr),
#else
    :
#endif
      driver(driver_yyarg) {
}

Parser::~Parser() {}

Parser::syntax_error::~syntax_error() YY_NOEXCEPT YY_NOTHROW {}

/*---------------.
| symbol kinds.  |
`---------------*/

// basic_symbol.
template <typename Base>
Parser::basic_symbol<Base>::basic_symbol(const basic_symbol& that)
    : Base(that), value(that.value), location(that.location) {}

/// Constructor for valueless symbols.
template <typename Base>
Parser::basic_symbol<Base>::basic_symbol(typename Base::kind_type t,
                                         YY_MOVE_REF(location_type) l)
    : Base(t), value(), location(l) {}

template <typename Base>
Parser::basic_symbol<Base>::basic_symbol(typename Base::kind_type t,
                                         YY_RVREF(semantic_type) v,
                                         YY_RVREF(location_type) l)
    : Base(t), value(YY_MOVE(v)), location(YY_MOVE(l)) {}

template <typename Base>
Parser::symbol_kind_type Parser::basic_symbol<Base>::type_get() const
    YY_NOEXCEPT {
  return this->kind();
}

template <typename Base>
bool Parser::basic_symbol<Base>::empty() const YY_NOEXCEPT {
  return this->kind() == symbol_kind::S_YYEMPTY;
}

template <typename Base>
void Parser::basic_symbol<Base>::move(basic_symbol& s) {
  super_type::move(s);
  value = YY_MOVE(s.value);
  location = YY_MOVE(s.location);
}

// by_kind.
Parser::by_kind::by_kind() : kind_(symbol_kind::S_YYEMPTY) {}

#if 201103L <= YY_CPLUSPLUS
Parser::by_kind::by_kind(by_kind&& that) : kind_(that.kind_) { that.clear(); }
#endif

Parser::by_kind::by_kind(const by_kind& that) : kind_(that.kind_) {}

Parser::by_kind::by_kind(token_kind_type t) : kind_(yytranslate_(t)) {}

void Parser::by_kind::clear() YY_NOEXCEPT { kind_ = symbol_kind::S_YYEMPTY; }

void Parser::by_kind::move(by_kind& that) {
  kind_ = that.kind_;
  that.clear();
}

Parser::symbol_kind_type Parser::by_kind::kind() const YY_NOEXCEPT {
  return kind_;
}

Parser::symbol_kind_type Parser::by_kind::type_get() const YY_NOEXCEPT {
  return this->kind();
}

// by_state.
Parser::by_state::by_state() YY_NOEXCEPT : state(empty_state) {}

Parser::by_state::by_state(const by_state& that) YY_NOEXCEPT
    : state(that.state) {}

void Parser::by_state::clear() YY_NOEXCEPT { state = empty_state; }

void Parser::by_state::move(by_state& that) {
  state = that.state;
  that.clear();
}

Parser::by_state::by_state(state_type s) YY_NOEXCEPT : state(s) {}

Parser::symbol_kind_type Parser::by_state::kind() const YY_NOEXCEPT {
  if (state == empty_state)
    return symbol_kind::S_YYEMPTY;
  else
    return YY_CAST(symbol_kind_type, yystos_[+state]);
}

Parser::stack_symbol_type::stack_symbol_type() {}

Parser::stack_symbol_type::stack_symbol_type(YY_RVREF(stack_symbol_type) that)
    : super_type(YY_MOVE(that.state), YY_MOVE(that.value),
                 YY_MOVE(that.location)) {
#if 201103L <= YY_CPLUSPLUS
  // that is emptied.
  that.state = empty_state;
#endif
}

Parser::stack_symbol_type::stack_symbol_type(state_type s,
                                             YY_MOVE_REF(symbol_type) that)
    : super_type(s, YY_MOVE(that.value), YY_MOVE(that.location)) {
  // that is emptied.
  that.kind_ = symbol_kind::S_YYEMPTY;
}

#if YY_CPLUSPLUS < 201103L
Parser::stack_symbol_type& Parser::stack_symbol_type::operator=(
    const stack_symbol_type& that) {
  state = that.state;
  value = that.value;
  location = that.location;
  return *this;
}

Parser::stack_symbol_type& Parser::stack_symbol_type::operator=(
    stack_symbol_type& that) {
  state = that.state;
  value = that.value;
  location = that.location;
  // that is emptied.
  that.state = empty_state;
  return *this;
}
#endif

template <typename Base>
void Parser::yy_destroy_(const char* yymsg, basic_symbol<Base>& yysym) const {
  if (yymsg) YY_SYMBOL_PRINT(yymsg, yysym);

  // User destructor.
  switch (yysym.kind()) {
    case symbol_kind::S_STRING:  // "string"
#line 79 "parser.yy"
    {
      delete (yysym.value.stringVal);
    }
#line 372 "bison_parser.cpp"
    break;

    case symbol_kind::S_add_se2:  // add_se2
#line 80 "parser.yy"
    {
      delete (yysym.value.commandNode);
    }
#line 378 "bison_parser.cpp"
    break;

    case symbol_kind::S_add_se3:  // add_se3
#line 80 "parser.yy"
    {
      delete (yysym.value.commandNode);
    }
#line 384 "bison_parser.cpp"
    break;

    case symbol_kind::S_fix_node:  // fix_node
#line 80 "parser.yy"
    {
      delete (yysym.value.commandNode);
    }
#line 390 "bison_parser.cpp"
    break;

    case symbol_kind::S_solve_state:  // solve_state
#line 80 "parser.yy"
    {
      delete (yysym.value.commandNode);
    }
#line 396 "bison_parser.cpp"
    break;

    case symbol_kind::S_query_state:  // query_state
#line 80 "parser.yy"
    {
      delete (yysym.value.commandNode);
    }
#line 402 "bison_parser.cpp"
    break;

    case symbol_kind::S_command:  // command
#line 80 "parser.yy"
    {
      delete (yysym.value.commandNode);
    }
#line 408 "bison_parser.cpp"
    break;

    default:
      break;
  }
}

#if SLAM_PARSERDEBUG
template <typename Base>
void Parser::yy_print_(std::ostream& yyo,
                       const basic_symbol<Base>& yysym) const {
  std::ostream& yyoutput = yyo;
  YY_USE(yyoutput);
  if (yysym.empty())
    yyo << "empty symbol";
  else {
    symbol_kind_type yykind = yysym.kind();
    yyo << (yykind < YYNTOKENS ? "token" : "nterm") << ' ' << yysym.name()
        << " (" << yysym.location << ": ";
    YY_USE(yykind);
    yyo << ')';
  }
}
#endif

void Parser::yypush_(const char* m, YY_MOVE_REF(stack_symbol_type) sym) {
  if (m) YY_SYMBOL_PRINT(m, sym);
  yystack_.push(YY_MOVE(sym));
}

void Parser::yypush_(const char* m, state_type s,
                     YY_MOVE_REF(symbol_type) sym) {
#if 201103L <= YY_CPLUSPLUS
  yypush_(m, stack_symbol_type(s, std::move(sym)));
#else
  stack_symbol_type ss(s, sym);
  yypush_(m, ss);
#endif
}

void Parser::yypop_(int n) { yystack_.pop(n); }

#if SLAM_PARSERDEBUG
std::ostream& Parser::debug_stream() const { return *yycdebug_; }

void Parser::set_debug_stream(std::ostream& o) { yycdebug_ = &o; }

Parser::debug_level_type Parser::debug_level() const { return yydebug_; }

void Parser::set_debug_level(debug_level_type l) { yydebug_ = l; }
#endif  // SLAM_PARSERDEBUG

Parser::state_type Parser::yy_lr_goto_state_(state_type yystate, int yysym) {
  int yyr = yypgoto_[yysym - YYNTOKENS] + yystate;
  if (0 <= yyr && yyr <= yylast_ && yycheck_[yyr] == yystate)
    return yytable_[yyr];
  else
    return yydefgoto_[yysym - YYNTOKENS];
}

bool Parser::yy_pact_value_is_default_(int yyvalue) {
  return yyvalue == yypact_ninf_;
}

bool Parser::yy_table_value_is_error_(int yyvalue) {
  return yyvalue == yytable_ninf_;
}

int Parser::operator()() { return parse(); }

int Parser::parse() {
  int yyn;
  /// Length of the RHS of the rule being reduced.
  int yylen = 0;

  // Error handling.
  int yynerrs_ = 0;
  int yyerrstatus_ = 0;

  /// The lookahead symbol.
  symbol_type yyla;

  /// The locations where the error started and ended.
  stack_symbol_type yyerror_range[3];

  /// The return value of parse ().
  int yyresult;

#if YY_EXCEPTIONS
  try
#endif  // YY_EXCEPTIONS
  {
    YYCDEBUG << "Starting parse\n";

    // User initialization code.
#line 38 "parser.yy"
    {
      // initialize the initial location object
      yyla.location.begin.filename = yyla.location.end.filename =
          &driver.streamname;
    }

#line 551 "bison_parser.cpp"

    /* Initialize the stack.  The initial state will be set in
       yynewstate, since the latter expects the semantical and the
       location values to have been already stored, initialize these
       stacks with a primary value.  */
    yystack_.clear();
    yypush_(YY_NULLPTR, 0, YY_MOVE(yyla));

  /*-----------------------------------------------.
  | yynewstate -- push a new symbol on the stack.  |
  `-----------------------------------------------*/
  yynewstate:
    YYCDEBUG << "Entering state " << int(yystack_[0].state) << '\n';
    YY_STACK_PRINT();

    // Accept?
    if (yystack_[0].state == yyfinal_) YYACCEPT;

    goto yybackup;

  /*-----------.
  | yybackup.  |
  `-----------*/
  yybackup:
    // Try to take a decision without lookahead.
    yyn = yypact_[+yystack_[0].state];
    if (yy_pact_value_is_default_(yyn)) goto yydefault;

    // Read a lookahead token.
    if (yyla.empty()) {
      YYCDEBUG << "Reading a token\n";
#if YY_EXCEPTIONS
      try
#endif  // YY_EXCEPTIONS
      {
        yyla.kind_ = yytranslate_(yylex(&yyla.value, &yyla.location));
      }
#if YY_EXCEPTIONS
      catch (const syntax_error& yyexc) {
        YYCDEBUG << "Caught exception: " << yyexc.what() << '\n';
        error(yyexc);
        goto yyerrlab1;
      }
#endif  // YY_EXCEPTIONS
    }
    YY_SYMBOL_PRINT("Next token is", yyla);

    if (yyla.kind() == symbol_kind::S_YYerror) {
      // The scanner already issued an error message, process directly
      // to error recovery.  But do not keep the error token as
      // lookahead, it is too special and may lead us to an endless
      // loop in error recovery. */
      yyla.kind_ = symbol_kind::S_YYUNDEF;
      goto yyerrlab1;
    }

    /* If the proper action on seeing token YYLA.TYPE is to reduce or
       to detect an error, take that action.  */
    yyn += yyla.kind();
    if (yyn < 0 || yylast_ < yyn || yycheck_[yyn] != yyla.kind()) {
      goto yydefault;
    }

    // Reduce or error.
    yyn = yytable_[yyn];
    if (yyn <= 0) {
      if (yy_table_value_is_error_(yyn)) goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }

    // Count tokens shifted since error; after three, turn off error status.
    if (yyerrstatus_) --yyerrstatus_;

    // Shift the lookahead token.
    yypush_("Shifting", state_type(yyn), YY_MOVE(yyla));
    goto yynewstate;

  /*-----------------------------------------------------------.
  | yydefault -- do the default action for the current state.  |
  `-----------------------------------------------------------*/
  yydefault:
    yyn = yydefact_[+yystack_[0].state];
    if (yyn == 0) goto yyerrlab;
    goto yyreduce;

  /*-----------------------------.
  | yyreduce -- do a reduction.  |
  `-----------------------------*/
  yyreduce:
    yylen = yyr2_[yyn];
    {
      stack_symbol_type yylhs;
      yylhs.state = yy_lr_goto_state_(yystack_[yylen].state, yyr1_[yyn]);
      /* If YYLEN is nonzero, implement the default value of the
         action: '$$ = $1'.  Otherwise, use the top of the stack.

         Otherwise, the following line sets YYLHS.VALUE to garbage.
         This behavior is undocumented and Bison users should not rely
         upon it.  */
      if (yylen)
        yylhs.value = yystack_[yylen - 1].value;
      else
        yylhs.value = yystack_[0].value;

      // Default location.
      {
        stack_type::slice range(yystack_, yylen);
        YYLLOC_DEFAULT(yylhs.location, range, yylen);
        yyerror_range[1].location = yylhs.location;
      }

      // Perform the reduction.
      YY_REDUCE_PRINT(yyn);
#if YY_EXCEPTIONS
      try
#endif  // YY_EXCEPTIONS
      {
        switch (yyn) {
          case 2:  // int_list: "integer"
#line 104 "parser.yy"
          {
            (yylhs.value.intList) = new std::vector<int>;
            (yylhs.value.intList)->push_back((yystack_[0].value.integerVal));
          }
#line 692 "bison_parser.cpp"
          break;

          case 3:  // int_list: int_list "integer"
#line 109 "parser.yy"
          {
            (yystack_[1].value.intList)
                ->push_back((yystack_[0].value.integerVal));
            (yylhs.value.intList) = (yystack_[1].value.intList);
          }
#line 701 "bison_parser.cpp"
          break;

          case 4:  // NUMBER: "integer"
#line 115 "parser.yy"
          {
            (yylhs.value.doubleVal) = (yystack_[0].value.integerVal);
          }
#line 709 "bison_parser.cpp"
          break;

          case 5:  // NUMBER: "double"
#line 119 "parser.yy"
          {
            (yylhs.value.doubleVal) = (yystack_[0].value.doubleVal);
          }
#line 717 "bison_parser.cpp"
          break;

          case 6:  // add_se2: "ADD" "Vertex SE2" "integer"
#line 124 "parser.yy"
          {
            (yylhs.value.commandNode) =
                new AddNode(*(yystack_[1].value.stringVal),
                            (yystack_[0].value.integerVal), 3);
            delete (yystack_[1].value.stringVal);
          }
#line 726 "bison_parser.cpp"
          break;

          case 7:  // add_se2: "ADD" "Vertex SE2" "integer" NUMBER NUMBER NUMBER
#line 129 "parser.yy"
          {
            std::vector<double> values;
            values.push_back((yystack_[2].value.doubleVal));
            values.push_back((yystack_[1].value.doubleVal));
            values.push_back((yystack_[0].value.doubleVal));
            (yylhs.value.commandNode) =
                new AddNode(*(yystack_[4].value.stringVal),
                            (yystack_[3].value.integerVal), 3, values);
            delete (yystack_[4].value.stringVal);
          }
#line 739 "bison_parser.cpp"
          break;

          case 8:  // add_se2: "ADD" "Edge SE2" "integer" "integer" "integer"
                   // NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER
                   // NUMBER
#line 138 "parser.yy"
          {
            std::vector<double> values;
            values.push_back((yystack_[8].value.doubleVal));
            values.push_back((yystack_[7].value.doubleVal));
            values.push_back((yystack_[6].value.doubleVal));
            std::vector<double> information;
            information.push_back((yystack_[5].value.doubleVal));
            information.push_back((yystack_[4].value.doubleVal));
            information.push_back((yystack_[3].value.doubleVal));
            information.push_back((yystack_[2].value.doubleVal));
            information.push_back((yystack_[1].value.doubleVal));
            information.push_back((yystack_[0].value.doubleVal));
            (yylhs.value.commandNode) = new AddEdge(
                *(yystack_[12].value.stringVal),
                (yystack_[11].value.integerVal), values.size(),
                (yystack_[10].value.integerVal), (yystack_[9].value.integerVal),
                values, information);
            delete (yystack_[12].value.stringVal);
          }
#line 759 "bison_parser.cpp"
          break;

          case 9:  // add_se3: "ADD" "Vertex SE3" "integer"
#line 155 "parser.yy"
          {
            (yylhs.value.commandNode) =
                new AddNode(*(yystack_[1].value.stringVal),
                            (yystack_[0].value.integerVal), 6);
            delete (yystack_[1].value.stringVal);
          }
#line 768 "bison_parser.cpp"
          break;

          case 10:  // add_se3: "ADD" "Vertex SE3" "integer" NUMBER NUMBER
                    // NUMBER NUMBER NUMBER NUMBER
#line 160 "parser.yy"
          {
            (yylhs.value.commandNode) =
                new AddNode(*(yystack_[7].value.stringVal),
                            (yystack_[6].value.integerVal), 6);
            delete (yystack_[7].value.stringVal);
          }
#line 777 "bison_parser.cpp"
          break;

          case 11:  // add_se3: "ADD" "Edge SE3" "integer" "integer" "integer"
                    // NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER
                    // NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER
                    // NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER
                    // NUMBER NUMBER NUMBER
#line 165 "parser.yy"
          {
            std::vector<double> values;
            values.push_back((yystack_[26].value.doubleVal));
            values.push_back((yystack_[25].value.doubleVal));
            values.push_back((yystack_[24].value.doubleVal));
            values.push_back((yystack_[23].value.doubleVal));
            values.push_back((yystack_[22].value.doubleVal));
            values.push_back((yystack_[21].value.doubleVal));
            std::vector<double> information;
            information.push_back((yystack_[20].value.doubleVal));
            information.push_back((yystack_[19].value.doubleVal));
            information.push_back((yystack_[18].value.doubleVal));
            information.push_back((yystack_[17].value.doubleVal));
            information.push_back((yystack_[16].value.doubleVal));
            information.push_back((yystack_[15].value.doubleVal));
            information.push_back((yystack_[14].value.doubleVal));
            information.push_back((yystack_[13].value.doubleVal));
            information.push_back((yystack_[12].value.doubleVal));
            information.push_back((yystack_[11].value.doubleVal));
            information.push_back((yystack_[10].value.doubleVal));
            information.push_back((yystack_[9].value.doubleVal));
            information.push_back((yystack_[8].value.doubleVal));
            information.push_back((yystack_[7].value.doubleVal));
            information.push_back((yystack_[6].value.doubleVal));
            information.push_back((yystack_[5].value.doubleVal));
            information.push_back((yystack_[4].value.doubleVal));
            information.push_back((yystack_[3].value.doubleVal));
            information.push_back((yystack_[2].value.doubleVal));
            information.push_back((yystack_[1].value.doubleVal));
            information.push_back((yystack_[0].value.doubleVal));
            (yylhs.value.commandNode) = new AddEdge(
                *(yystack_[30].value.stringVal),
                (yystack_[29].value.integerVal), values.size(),
                (yystack_[28].value.integerVal),
                (yystack_[27].value.integerVal), values, information);
            delete (yystack_[30].value.stringVal);
          }
#line 815 "bison_parser.cpp"
          break;

          case 12:  // fix_node: "Fix" "integer"
#line 200 "parser.yy"
          {
            std::vector<int> values;
            values.push_back((yystack_[0].value.integerVal));
            (yylhs.value.commandNode) = new FixNode("FIX", values);
          }
#line 825 "bison_parser.cpp"
          break;

          case 13:  // solve_state: "Solve State"
#line 207 "parser.yy"
          {
            (yylhs.value.commandNode) = new SolveSate("SOLVE_STATE");
          }
#line 833 "bison_parser.cpp"
          break;

          case 14:  // query_state: "Query State"
#line 212 "parser.yy"
          {
            (yylhs.value.commandNode) = new QueryState("QUERY_STATE");
          }
#line 841 "bison_parser.cpp"
          break;

          case 15:  // query_state: "Query State" int_list
#line 216 "parser.yy"
          {
            (yylhs.value.commandNode) =
                new QueryState("QUERY_STATE", *(yystack_[0].value.intList));
            delete (yystack_[0].value.intList);
          }
#line 850 "bison_parser.cpp"
          break;

          case 16:  // command: add_se2
#line 222 "parser.yy"
          {
            (yylhs.value.commandNode) = (yystack_[0].value.commandNode);
          }
#line 858 "bison_parser.cpp"
          break;

          case 17:  // command: add_se3
#line 226 "parser.yy"
          {
            (yylhs.value.commandNode) = (yystack_[0].value.commandNode);
          }
#line 866 "bison_parser.cpp"
          break;

          case 18:  // command: fix_node
#line 230 "parser.yy"
          {
            (yylhs.value.commandNode) = (yystack_[0].value.commandNode);
          }
#line 874 "bison_parser.cpp"
          break;

          case 19:  // command: solve_state
#line 234 "parser.yy"
          {
            (yylhs.value.commandNode) = (yystack_[0].value.commandNode);
          }
#line 882 "bison_parser.cpp"
          break;

          case 20:  // command: query_state
#line 238 "parser.yy"
          {
            (yylhs.value.commandNode) = (yystack_[0].value.commandNode);
          }
#line 890 "bison_parser.cpp"
          break;

          case 23:  // start: start command ';'
#line 245 "parser.yy"
          {
            driver.slamContext.process((yystack_[1].value.commandNode));
            delete (yystack_[1].value.commandNode);
          }
#line 899 "bison_parser.cpp"
          break;

#line 903 "bison_parser.cpp"

          default:
            break;
        }
      }
#if YY_EXCEPTIONS
      catch (const syntax_error& yyexc) {
        YYCDEBUG << "Caught exception: " << yyexc.what() << '\n';
        error(yyexc);
        YYERROR;
      }
#endif  // YY_EXCEPTIONS
      YY_SYMBOL_PRINT("-> $$ =", yylhs);
      yypop_(yylen);
      yylen = 0;

      // Shift the result of the reduction.
      yypush_(YY_NULLPTR, YY_MOVE(yylhs));
    }
    goto yynewstate;

  /*--------------------------------------.
  | yyerrlab -- here on detecting error.  |
  `--------------------------------------*/
  yyerrlab:
    // If not already recovering from an error, report this error.
    if (!yyerrstatus_) {
      ++yynerrs_;
      context yyctx(*this, yyla);
      std::string msg = yysyntax_error_(yyctx);
      error(yyla.location, YY_MOVE(msg));
    }

    yyerror_range[1].location = yyla.location;
    if (yyerrstatus_ == 3) {
      /* If just tried and failed to reuse lookahead token after an
         error, discard it.  */

      // Return failure if at end of input.
      if (yyla.kind() == symbol_kind::S_YYEOF)
        YYABORT;
      else if (!yyla.empty()) {
        yy_destroy_("Error: discarding", yyla);
        yyla.clear();
      }
    }

    // Else will try to reuse lookahead token after shifting the error token.
    goto yyerrlab1;

  /*---------------------------------------------------.
  | yyerrorlab -- error raised explicitly by YYERROR.  |
  `---------------------------------------------------*/
  yyerrorlab:
    /* Pacify compilers when the user code never invokes YYERROR and
       the label yyerrorlab therefore never appears in user code.  */
    if (false) YYERROR;

    /* Do not reclaim the symbols of the rule whose action triggered
       this YYERROR.  */
    yypop_(yylen);
    yylen = 0;
    YY_STACK_PRINT();
    goto yyerrlab1;

  /*-------------------------------------------------------------.
  | yyerrlab1 -- common code for both syntax error and YYERROR.  |
  `-------------------------------------------------------------*/
  yyerrlab1:
    yyerrstatus_ = 3;  // Each real token shifted decrements this.
    // Pop stack until we find a state that shifts the error token.
    for (;;) {
      yyn = yypact_[+yystack_[0].state];
      if (!yy_pact_value_is_default_(yyn)) {
        yyn += symbol_kind::S_YYerror;
        if (0 <= yyn && yyn <= yylast_ &&
            yycheck_[yyn] == symbol_kind::S_YYerror) {
          yyn = yytable_[yyn];
          if (0 < yyn) break;
        }
      }

      // Pop the current state because it cannot handle the error token.
      if (yystack_.size() == 1) YYABORT;

      yyerror_range[1].location = yystack_[0].location;
      yy_destroy_("Error: popping", yystack_[0]);
      yypop_();
      YY_STACK_PRINT();
    }
    {
      stack_symbol_type error_token;

      yyerror_range[2].location = yyla.location;
      YYLLOC_DEFAULT(error_token.location, yyerror_range, 2);

      // Shift the error token.
      error_token.state = state_type(yyn);
      yypush_("Shifting", YY_MOVE(error_token));
    }
    goto yynewstate;

  /*-------------------------------------.
  | yyacceptlab -- YYACCEPT comes here.  |
  `-------------------------------------*/
  yyacceptlab:
    yyresult = 0;
    goto yyreturn;

  /*-----------------------------------.
  | yyabortlab -- YYABORT comes here.  |
  `-----------------------------------*/
  yyabortlab:
    yyresult = 1;
    goto yyreturn;

  /*-----------------------------------------------------.
  | yyreturn -- parsing is finished, return the result.  |
  `-----------------------------------------------------*/
  yyreturn:
    if (!yyla.empty()) yy_destroy_("Cleanup: discarding lookahead", yyla);

    /* Do not reclaim the symbols of the rule whose action triggered
       this YYABORT or YYACCEPT.  */
    yypop_(yylen);
    YY_STACK_PRINT();
    while (1 < yystack_.size()) {
      yy_destroy_("Cleanup: popping", yystack_[0]);
      yypop_();
    }

    return yyresult;
  }
#if YY_EXCEPTIONS
  catch (...) {
    YYCDEBUG << "Exception caught: cleaning lookahead and stack\n";
    // Do not try to display the values of the reclaimed symbols,
    // as their printers might throw an exception.
    if (!yyla.empty()) yy_destroy_(YY_NULLPTR, yyla);

    while (1 < yystack_.size()) {
      yy_destroy_(YY_NULLPTR, yystack_[0]);
      yypop_();
    }
    throw;
  }
#endif  // YY_EXCEPTIONS
}

void Parser::error(const syntax_error& yyexc) {
  error(yyexc.location, yyexc.what());
}

/* Return YYSTR after stripping away unnecessary quotes and
   backslashes, so that it's suitable for yyerror.  The heuristic is
   that double-quoting is unnecessary unless the string contains an
   apostrophe, a comma, or backslash (other than backslash-backslash).
   YYSTR is taken from yytname.  */
std::string Parser::yytnamerr_(const char* yystr) {
  if (*yystr == '"') {
    std::string yyr;
    char const* yyp = yystr;

    for (;;) switch (*++yyp) {
        case '\'':
        case ',':
          goto do_not_strip_quotes;

        case '\\':
          if (*++yyp != '\\')
            goto do_not_strip_quotes;
          else
            goto append;

        append:
        default:
          yyr += *yyp;
          break;

        case '"':
          return yyr;
      }
  do_not_strip_quotes:;
  }

  return yystr;
}

std::string Parser::symbol_name(symbol_kind_type yysymbol) {
  return yytnamerr_(yytname_[yysymbol]);
}

// Parser::context.
Parser::context::context(const Parser& yyparser, const symbol_type& yyla)
    : yyparser_(yyparser), yyla_(yyla) {}

int Parser::context::expected_tokens(symbol_kind_type yyarg[],
                                     int yyargn) const {
  // Actual number of expected tokens
  int yycount = 0;

  int yyn = yypact_[+yyparser_.yystack_[0].state];
  if (!yy_pact_value_is_default_(yyn)) {
    /* Start YYX at -YYN if negative to avoid negative indexes in
       YYCHECK.  In other words, skip the first -YYN actions for
       this state because they are default actions.  */
    int yyxbegin = yyn < 0 ? -yyn : 0;
    // Stay within bounds of both yycheck and yytname.
    int yychecklim = yylast_ - yyn + 1;
    int yyxend = yychecklim < YYNTOKENS ? yychecklim : YYNTOKENS;
    for (int yyx = yyxbegin; yyx < yyxend; ++yyx)
      if (yycheck_[yyx + yyn] == yyx && yyx != symbol_kind::S_YYerror &&
          !yy_table_value_is_error_(yytable_[yyx + yyn])) {
        if (!yyarg)
          ++yycount;
        else if (yycount == yyargn)
          return 0;
        else
          yyarg[yycount++] = YY_CAST(symbol_kind_type, yyx);
      }
  }

  if (yyarg && yycount == 0 && 0 < yyargn) yyarg[0] = symbol_kind::S_YYEMPTY;
  return yycount;
}

int Parser::yy_syntax_error_arguments_(const context& yyctx,
                                       symbol_kind_type yyarg[],
                                       int yyargn) const {
  /* There are many possibilities here to consider:
     - If this state is a consistent state with a default action, then
       the only way this function was invoked is if the default action
       is an error action.  In that case, don't check for expected
       tokens because there are none.
     - The only way there can be no lookahead present (in yyla) is
       if this state is a consistent state with a default action.
       Thus, detecting the absence of a lookahead is sufficient to
       determine that there is no unexpected or expected token to
       report.  In that case, just report a simple "syntax error".
     - Don't assume there isn't a lookahead just because this state is
       a consistent state with a default action.  There might have
       been a previous inconsistent state, consistent state with a
       non-default action, or user semantic action that manipulated
       yyla.  (However, yyla is currently not documented for users.)
     - Of course, the expected token list depends on states to have
       correct lookahead information, and it depends on the parser not
       to perform extra reductions after fetching a lookahead from the
       scanner and before detecting a syntax error.  Thus, state merging
       (from LALR or IELR) and default reductions corrupt the expected
       token list.  However, the list is correct for canonical LR with
       one exception: it will still contain any token that will not be
       accepted due to an error action in a later state.
  */

  if (!yyctx.lookahead().empty()) {
    if (yyarg) yyarg[0] = yyctx.token();
    int yyn = yyctx.expected_tokens(yyarg ? yyarg + 1 : yyarg, yyargn - 1);
    return yyn + 1;
  }
  return 0;
}

// Generate an error message.
std::string Parser::yysyntax_error_(const context& yyctx) const {
  // Its maximum.
  enum { YYARGS_MAX = 5 };
  // Arguments of yyformat.
  symbol_kind_type yyarg[YYARGS_MAX];
  int yycount = yy_syntax_error_arguments_(yyctx, yyarg, YYARGS_MAX);

  char const* yyformat = YY_NULLPTR;
  switch (yycount) {
#define YYCASE_(N, S) \
  case N:             \
    yyformat = S;     \
    break
    default:  // Avoid compiler warnings.
      YYCASE_(0, YY_("syntax error"));
      YYCASE_(1, YY_("syntax error, unexpected %s"));
      YYCASE_(2, YY_("syntax error, unexpected %s, expecting %s"));
      YYCASE_(3, YY_("syntax error, unexpected %s, expecting %s or %s"));
      YYCASE_(4, YY_("syntax error, unexpected %s, expecting %s or %s or %s"));
      YYCASE_(
          5,
          YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s"));
#undef YYCASE_
  }

  std::string yyres;
  // Argument number.
  std::ptrdiff_t yyi = 0;
  for (char const* yyp = yyformat; *yyp; ++yyp)
    if (yyp[0] == '%' && yyp[1] == 's' && yyi < yycount) {
      yyres += symbol_name(yyarg[yyi++]);
      ++yyp;
    } else
      yyres += *yyp;
  return yyres;
}

const signed char Parser::yypact_ninf_ = -24;

const signed char Parser::yytable_ninf_ = -1;

const signed char Parser::yypact_[] = {
    -24, 54, -24, -24, -7, 1,   -24, 10, -24, -24, -24, -24, -24, 9,  31, 51,
    52,  55, -24, -24, 56, -24, 4,   4,  58,  59,  -24, -24, -24, 4,  4,  60,
    61,  4,  4,   4,   4,  -24, 4,   4,  4,   4,   4,   4,   4,   4,  4,  -24,
    4,   4,  4,   4,   4,  4,   4,   4,  4,   4,   -24, 4,   4,   4,  4,  4,
    4,   4,  4,   4,   4,  4,   4,   4,  4,   4,   4,   4,   4,   -24};

const signed char Parser::yydefact_[] = {
    21, 0,  1, 22, 0, 0, 13, 14, 16, 17, 18, 19, 20, 0, 0, 0, 0, 0, 12, 2,
    15, 23, 6, 9,  0, 0, 3,  4,  5,  0,  0,  0,  0,  0, 0, 0, 0, 7, 0,  0,
    0,  0,  0, 0,  0, 0, 0,  10, 0,  0,  0,  0,  0,  0, 0, 0, 0, 0, 8,  0,
    0,  0,  0, 0,  0, 0, 0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 11};

const signed char Parser::yypgoto_[] = {-24, -24, -23, -24, -24,
                                        -24, -24, -24, -24, -24};

const signed char Parser::yydefgoto_[] = {0, 20, 29, 8, 9, 10, 11, 12, 13, 1};

const signed char Parser::yytable_[] = {
    30, 14, 15, 16, 17, 18, 33, 34, 27, 28, 37, 38, 39, 40, 19, 41, 42, 43,
    44, 45, 46, 47, 48, 49, 21, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 22,
    60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77,
    2,  23, 24, 3,  0,  25, 26, 4,  31, 32, 35, 36, 5,  6,  7};

const signed char Parser::yycheck_[] = {
    23, 8,  9,  10, 11, 4,  29, 30, 4,  5,  33, 34, 35, 36, 4,  38, 39, 40,
    41, 42, 43, 44, 45, 46, 15, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 4,
    59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76,
    0,  4,  4,  3,  -1, 4,  4,  7,  4,  4,  4,  4,  12, 13, 14};

const signed char Parser::yystos_[] = {
    0,  25, 0,  3,  7,  12, 13, 14, 19, 20, 21, 22, 23, 24, 8,  9,
    10, 11, 4,  4,  17, 15, 4,  4,  4,  4,  4,  4,  5,  18, 18, 4,
    4,  18, 18, 4,  4,  18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
    18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
    18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18};

const signed char Parser::yyr1_[] = {0,  16, 17, 17, 18, 18, 19, 19,
                                     19, 20, 20, 20, 21, 22, 23, 23,
                                     24, 24, 24, 24, 24, 25, 25, 25};

const signed char Parser::yyr2_[] = {0, 2, 1, 2, 1, 1, 3, 6, 14, 3, 9, 32,
                                     2, 1, 1, 2, 1, 1, 1, 1, 1,  0, 2, 3};

#if SLAM_PARSERDEBUG || 1
// YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
// First, the terminals, then, starting at \a YYNTOKENS, nonterminals.
const char* const Parser::yytname_[] = {"\"end of file\"",
                                        "error",
                                        "\"invalid token\"",
                                        "\"end of line\"",
                                        "\"integer\"",
                                        "\"double\"",
                                        "\"string\"",
                                        "\"ADD\"",
                                        "\"Vertex SE2\"",
                                        "\"Vertex SE3\"",
                                        "\"Edge SE2\"",
                                        "\"Edge SE3\"",
                                        "\"Fix\"",
                                        "\"Solve State\"",
                                        "\"Query State\"",
                                        "';'",
                                        "$accept",
                                        "int_list",
                                        "NUMBER",
                                        "add_se2",
                                        "add_se3",
                                        "fix_node",
                                        "solve_state",
                                        "query_state",
                                        "command",
                                        "start",
                                        YY_NULLPTR};
#endif

#if SLAM_PARSERDEBUG
const unsigned char Parser::yyrline_[] = {
    0,   103, 103, 108, 114, 118, 123, 128, 137, 154, 159, 164,
    199, 206, 211, 215, 221, 225, 229, 233, 237, 242, 243, 244};

void Parser::yy_stack_print_() const {
  *yycdebug_ << "Stack now";
  for (stack_type::const_iterator i = yystack_.begin(), i_end = yystack_.end();
       i != i_end; ++i)
    *yycdebug_ << ' ' << int(i->state);
  *yycdebug_ << '\n';
}

void Parser::yy_reduce_print_(int yyrule) const {
  int yylno = yyrline_[yyrule];
  int yynrhs = yyr2_[yyrule];
  // Print the symbols being reduced, and their result.
  *yycdebug_ << "Reducing stack by rule " << yyrule - 1 << " (line " << yylno
             << "):\n";
  // The symbols being reduced.
  for (int yyi = 0; yyi < yynrhs; yyi++)
    YY_SYMBOL_PRINT("   $" << yyi + 1 << " =", yystack_[(yynrhs) - (yyi + 1)]);
}
#endif  // SLAM_PARSERDEBUG

Parser::symbol_kind_type Parser::yytranslate_(int t) {
  // YYTRANSLATE[TOKEN-NUM] -- Symbol number corresponding to
  // TOKEN-NUM as returned by yylex.
  static const signed char translate_table[] = {
      0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,  2,  2,  2,  2, 2, 2, 2, 2, 2, 2,
      2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,  2,  2,  2,  2, 2, 2, 2, 2, 2, 2,
      2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,  15, 2,  2,  2, 2, 2, 2, 2, 2, 2,
      2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,  2,  2,  2,  2, 2, 2, 2, 2, 2, 2,
      2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,  2,  2,  2,  2, 2, 2, 2, 2, 2, 2,
      2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,  2,  2,  2,  2, 2, 2, 2, 2, 2, 2,
      2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,  2,  2,  2,  2, 2, 2, 2, 2, 2, 2,
      2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,  2,  2,  2,  2, 2, 2, 2, 2, 2, 2,
      2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,  2,  2,  2,  2, 2, 2, 2, 2, 2, 2,
      2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,  2,  2,  2,  2, 2, 2, 2, 2, 2, 2,
      2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,  2,  2,  2,  2, 2, 2, 2, 2, 2, 2,
      2, 2, 2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
  // Last valid token kind.
  const int code_max = 269;

  if (t <= 0)
    return symbol_kind::S_YYEOF;
  else if (t <= code_max)
    return YY_CAST(symbol_kind_type, translate_table[t]);
  else
    return symbol_kind::S_YYUNDEF;
}

}  // namespace slam_parser
#line 1449 "bison_parser.cpp"

#line 252 "parser.yy"
/*** Additional Code ***/

void slam_parser::Parser::error(const Parser::location_type& l,
                                const std::string& m) {
  driver.error(l, m);
}
