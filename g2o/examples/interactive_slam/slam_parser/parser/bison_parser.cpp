
/* A Bison parser, made by GNU Bison 2.4.1.  */

/* Skeleton implementation for Bison LALR(1) parsers in C++
   
      Copyright (C) 2002, 2003, 2004, 2005, 2006, 2007, 2008 Free Software
   Foundation, Inc.
   
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.
   
   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

// Take the name prefix into account.
#define yylex   SlamParserlex

/* First part of user declarations.  */

/* Line 311 of lalr1.cc  */
#line 1 "parser.yy"
 /*** C/C++ Declarations ***/

#include <stdio.h>
#include <string>
#include <vector>

#include "commands.h"



/* Line 311 of lalr1.cc  */
#line 54 "bison_parser.cpp"


#include "bison_parser.h"

/* User implementation prologue.  */

/* Line 317 of lalr1.cc  */
#line 85 "parser.yy"


#include "driver.h"
#include "scanner.h"
#include "slam_context.h"

/* this "connects" the bison parser in the driver to the flex scanner class
 * object. it defines the yylex() function call to pull the next token from the
 * current lexer object of the driver context. */
#undef yylex
#define yylex driver.lexer->lex



/* Line 317 of lalr1.cc  */
#line 78 "bison_parser.cpp"

#ifndef YY_
# if YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* FIXME: INFRINGES ON USER NAME SPACE */
#   define YY_(msgid) dgettext ("bison-runtime", msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(msgid) msgid
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#define YYUSE(e) ((void) (e))

/* Enable debugging if requested.  */
#if YYDEBUG

/* A pseudo ostream that takes yydebug_ into account.  */
# define YYCDEBUG if (yydebug_) (*yycdebug_)

# define YY_SYMBOL_PRINT(Title, Type, Value, Location)  \
do {              \
  if (yydebug_)            \
    {              \
      *yycdebug_ << Title << ' ';      \
      yy_symbol_print_ ((Type), (Value), (Location));  \
      *yycdebug_ << std::endl;        \
    }              \
} while (false)

# define YY_REDUCE_PRINT(Rule)    \
do {          \
  if (yydebug_)        \
    yy_reduce_print_ (Rule);    \
} while (false)

# define YY_STACK_PRINT()    \
do {          \
  if (yydebug_)        \
    yystack_print_ ();      \
} while (false)

#else /* !YYDEBUG */

# define YYCDEBUG if (false) std::cerr
# define YY_SYMBOL_PRINT(Title, Type, Value, Location)
# define YY_REDUCE_PRINT(Rule)
# define YY_STACK_PRINT()

#endif /* !YYDEBUG */

#define yyerrok    (yyerrstatus_ = 0)
#define yyclearin  (yychar = yyempty_)

#define YYACCEPT  goto yyacceptlab
#define YYABORT    goto yyabortlab
#define YYERROR    goto yyerrorlab
#define YYRECOVERING()  (!!yyerrstatus_)


/* Line 380 of lalr1.cc  */
#line 1 "[Bison:b4_percent_define_default]"

namespace SlamParser {

/* Line 380 of lalr1.cc  */
#line 147 "bison_parser.cpp"
#if YYERROR_VERBOSE

  /* Return YYSTR after stripping away unnecessary quotes and
     backslashes, so that it's suitable for yyerror.  The heuristic is
     that double-quoting is unnecessary unless the string contains an
     apostrophe, a comma, or backslash (other than backslash-backslash).
     YYSTR is taken from yytname.  */
  std::string
  Parser::yytnamerr_ (const char *yystr)
  {
    if (*yystr == '"')
      {
        std::string yyr = "";
        char const *yyp = yystr;

        for (;;)
          switch (*++yyp)
            {
            case '\'':
            case ',':
              goto do_not_strip_quotes;

            case '\\':
              if (*++yyp != '\\')
                goto do_not_strip_quotes;
              /* Fall through.  */
            default:
              yyr += *yyp;
              break;

            case '"':
              return yyr;
            }
      do_not_strip_quotes: ;
      }

    return yystr;
  }

#endif

  /// Build a parser object.
  Parser::Parser (class Driver& driver_yyarg)
    :
#if YYDEBUG
      yydebug_ (false),
      yycdebug_ (&std::cerr),
#endif
      driver (driver_yyarg)
  {
  }

  Parser::~Parser ()
  {
  }

#if YYDEBUG
  /*--------------------------------.
  | Print this symbol on YYOUTPUT.  |
  `--------------------------------*/

  inline void
  Parser::yy_symbol_value_print_ (int yytype,
         const semantic_type* yyvaluep, const location_type* yylocationp)
  {
    YYUSE (yylocationp);
    YYUSE (yyvaluep);
    switch (yytype)
      {
         default:
    break;
      }
  }


  void
  Parser::yy_symbol_print_ (int yytype,
         const semantic_type* yyvaluep, const location_type* yylocationp)
  {
    *yycdebug_ << (yytype < yyntokens_ ? "token" : "nterm")
         << ' ' << yytname_[yytype] << " ("
         << *yylocationp << ": ";
    yy_symbol_value_print_ (yytype, yyvaluep, yylocationp);
    *yycdebug_ << ')';
  }
#endif

  void
  Parser::yydestruct_ (const char* yymsg,
         int yytype, semantic_type* yyvaluep, location_type* yylocationp)
  {
    YYUSE (yylocationp);
    YYUSE (yymsg);
    YYUSE (yyvaluep);

    YY_SYMBOL_PRINT (yymsg, yytype, yyvaluep, yylocationp);

    switch (yytype)
      {
        case 6: /* "\"string\"" */

/* Line 480 of lalr1.cc  */
#line 79 "parser.yy"
  { delete (yyvaluep->stringVal); };

/* Line 480 of lalr1.cc  */
#line 254 "bison_parser.cpp"
  break;
      case 19: /* "add_se2" */

/* Line 480 of lalr1.cc  */
#line 80 "parser.yy"
  { delete (yyvaluep->commandNode); };

/* Line 480 of lalr1.cc  */
#line 263 "bison_parser.cpp"
  break;
      case 20: /* "add_se3" */

/* Line 480 of lalr1.cc  */
#line 80 "parser.yy"
  { delete (yyvaluep->commandNode); };

/* Line 480 of lalr1.cc  */
#line 272 "bison_parser.cpp"
  break;
      case 21: /* "fix_node" */

/* Line 480 of lalr1.cc  */
#line 80 "parser.yy"
  { delete (yyvaluep->commandNode); };

/* Line 480 of lalr1.cc  */
#line 281 "bison_parser.cpp"
  break;
      case 22: /* "solve_state" */

/* Line 480 of lalr1.cc  */
#line 80 "parser.yy"
  { delete (yyvaluep->commandNode); };

/* Line 480 of lalr1.cc  */
#line 290 "bison_parser.cpp"
  break;
      case 23: /* "query_state" */

/* Line 480 of lalr1.cc  */
#line 80 "parser.yy"
  { delete (yyvaluep->commandNode); };

/* Line 480 of lalr1.cc  */
#line 299 "bison_parser.cpp"
  break;
      case 24: /* "command" */

/* Line 480 of lalr1.cc  */
#line 80 "parser.yy"
  { delete (yyvaluep->commandNode); };

/* Line 480 of lalr1.cc  */
#line 308 "bison_parser.cpp"
  break;

  default:
    break;
      }
  }

  void
  Parser::yypop_ (unsigned int n)
  {
    yystate_stack_.pop (n);
    yysemantic_stack_.pop (n);
    yylocation_stack_.pop (n);
  }

#if YYDEBUG
  std::ostream&
  Parser::debug_stream () const
  {
    return *yycdebug_;
  }

  void
  Parser::set_debug_stream (std::ostream& o)
  {
    yycdebug_ = &o;
  }


  Parser::debug_level_type
  Parser::debug_level () const
  {
    return yydebug_;
  }

  void
  Parser::set_debug_level (debug_level_type l)
  {
    yydebug_ = l;
  }
#endif

  int
  Parser::parse ()
  {
    /// Lookahead and lookahead in internal form.
    int yychar = yyempty_;
    int yytoken = 0;

    /* State.  */
    int yyn;
    int yylen = 0;
    int yystate = 0;

    /* Error handling.  */
    int yynerrs_ = 0;
    int yyerrstatus_ = 0;

    /// Semantic value of the lookahead.
    semantic_type yylval;
    /// Location of the lookahead.
    location_type yylloc;
    /// The locations where the error started and ended.
    location_type yyerror_range[2];

    /// $$.
    semantic_type yyval;
    /// @$.
    location_type yyloc;

    int yyresult;

    YYCDEBUG << "Starting parse" << std::endl;


    /* User initialization code.  */
    
/* Line 553 of lalr1.cc  */
#line 38 "parser.yy"
{
    // initialize the initial location object
    yylloc.begin.filename = yylloc.end.filename = &driver.streamname;
}

/* Line 553 of lalr1.cc  */
#line 394 "bison_parser.cpp"

    /* Initialize the stacks.  The initial state will be pushed in
       yynewstate, since the latter expects the semantical and the
       location values to have been already stored, initialize these
       stacks with a primary value.  */
    yystate_stack_ = state_stack_type (0);
    yysemantic_stack_ = semantic_stack_type (0);
    yylocation_stack_ = location_stack_type (0);
    yysemantic_stack_.push (yylval);
    yylocation_stack_.push (yylloc);

    /* New state.  */
  yynewstate:
    yystate_stack_.push (yystate);
    YYCDEBUG << "Entering state " << yystate << std::endl;

    /* Accept?  */
    if (yystate == yyfinal_)
      goto yyacceptlab;

    goto yybackup;

    /* Backup.  */
  yybackup:

    /* Try to take a decision without lookahead.  */
    yyn = yypact_[yystate];
    if (yyn == yypact_ninf_)
      goto yydefault;

    /* Read a lookahead token.  */
    if (yychar == yyempty_)
      {
  YYCDEBUG << "Reading a token: ";
  yychar = yylex (&yylval, &yylloc);
      }


    /* Convert token to internal form.  */
    if (yychar <= yyeof_)
      {
  yychar = yytoken = yyeof_;
  YYCDEBUG << "Now at end of input." << std::endl;
      }
    else
      {
  yytoken = yytranslate_ (yychar);
  YY_SYMBOL_PRINT ("Next token is", yytoken, &yylval, &yylloc);
      }

    /* If the proper action on seeing token YYTOKEN is to reduce or to
       detect an error, take that action.  */
    yyn += yytoken;
    if (yyn < 0 || yylast_ < yyn || yycheck_[yyn] != yytoken)
      goto yydefault;

    /* Reduce or error.  */
    yyn = yytable_[yyn];
    if (yyn <= 0)
      {
  if (yyn == 0 || yyn == yytable_ninf_)
  goto yyerrlab;
  yyn = -yyn;
  goto yyreduce;
      }

    /* Shift the lookahead token.  */
    YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);

    /* Discard the token being shifted.  */
    yychar = yyempty_;

    yysemantic_stack_.push (yylval);
    yylocation_stack_.push (yylloc);

    /* Count tokens shifted since error; after three, turn off error
       status.  */
    if (yyerrstatus_)
      --yyerrstatus_;

    yystate = yyn;
    goto yynewstate;

  /*-----------------------------------------------------------.
  | yydefault -- do the default action for the current state.  |
  `-----------------------------------------------------------*/
  yydefault:
    yyn = yydefact_[yystate];
    if (yyn == 0)
      goto yyerrlab;
    goto yyreduce;

  /*-----------------------------.
  | yyreduce -- Do a reduction.  |
  `-----------------------------*/
  yyreduce:
    yylen = yyr2_[yyn];
    /* If YYLEN is nonzero, implement the default value of the action:
       `$$ = $1'.  Otherwise, use the top of the stack.

       Otherwise, the following line sets YYVAL to garbage.
       This behavior is undocumented and Bison
       users should not rely upon it.  */
    if (yylen)
      yyval = yysemantic_stack_[yylen - 1];
    else
      yyval = yysemantic_stack_[0];

    {
      slice<location_type, location_stack_type> slice (yylocation_stack_, yylen);
      YYLLOC_DEFAULT (yyloc, slice, yylen);
    }
    YY_REDUCE_PRINT (yyn);
    switch (yyn)
      {
    case 2:

/* Line 678 of lalr1.cc  */
#line 104 "parser.yy"
    {
              (yyval.intList) = new std::vector<int>;
              (yyval.intList)->push_back((yysemantic_stack_[(1) - (1)].integerVal));
           }
    break;

  case 3:

/* Line 678 of lalr1.cc  */
#line 109 "parser.yy"
    {
             (yysemantic_stack_[(2) - (1)].intList)->push_back((yysemantic_stack_[(2) - (2)].integerVal));
             (yyval.intList) = (yysemantic_stack_[(2) - (1)].intList); 
           }
    break;

  case 4:

/* Line 678 of lalr1.cc  */
#line 115 "parser.yy"
    {
     (yyval.doubleVal) = (yysemantic_stack_[(1) - (1)].integerVal);
   }
    break;

  case 5:

/* Line 678 of lalr1.cc  */
#line 119 "parser.yy"
    {
     (yyval.doubleVal) = (yysemantic_stack_[(1) - (1)].doubleVal);
   }
    break;

  case 6:

/* Line 678 of lalr1.cc  */
#line 124 "parser.yy"
    {
         (yyval.commandNode) = new AddNode(*(yysemantic_stack_[(3) - (2)].stringVal), (yysemantic_stack_[(3) - (3)].integerVal), 3);
               delete (yysemantic_stack_[(3) - (2)].stringVal);
     }
    break;

  case 7:

/* Line 678 of lalr1.cc  */
#line 129 "parser.yy"
    {
              std::vector<double> values;
              values.push_back((yysemantic_stack_[(6) - (4)].doubleVal));
              values.push_back((yysemantic_stack_[(6) - (5)].doubleVal));
              values.push_back((yysemantic_stack_[(6) - (6)].doubleVal));
        (yyval.commandNode) = new AddNode(*(yysemantic_stack_[(6) - (2)].stringVal), (yysemantic_stack_[(6) - (3)].integerVal), 3, values);
              delete (yysemantic_stack_[(6) - (2)].stringVal);
     }
    break;

  case 8:

/* Line 678 of lalr1.cc  */
#line 138 "parser.yy"
    {
              std::vector<double> values;
              values.push_back((yysemantic_stack_[(14) - (6)].doubleVal));
              values.push_back((yysemantic_stack_[(14) - (7)].doubleVal));
              values.push_back((yysemantic_stack_[(14) - (8)].doubleVal));
              std::vector<double> information;
              information.push_back((yysemantic_stack_[(14) - (9)].doubleVal));
              information.push_back((yysemantic_stack_[(14) - (10)].doubleVal));
              information.push_back((yysemantic_stack_[(14) - (11)].doubleVal));
              information.push_back((yysemantic_stack_[(14) - (12)].doubleVal));
              information.push_back((yysemantic_stack_[(14) - (13)].doubleVal));
              information.push_back((yysemantic_stack_[(14) - (14)].doubleVal));
              (yyval.commandNode) = new AddEdge(*(yysemantic_stack_[(14) - (2)].stringVal), (yysemantic_stack_[(14) - (3)].integerVal), values.size(), (yysemantic_stack_[(14) - (4)].integerVal), (yysemantic_stack_[(14) - (5)].integerVal), values, information);
              delete (yysemantic_stack_[(14) - (2)].stringVal);
     }
    break;

  case 9:

/* Line 678 of lalr1.cc  */
#line 155 "parser.yy"
    {
               (yyval.commandNode) = new AddNode(*(yysemantic_stack_[(3) - (2)].stringVal), (yysemantic_stack_[(3) - (3)].integerVal), 6);
               delete (yysemantic_stack_[(3) - (2)].stringVal);
     }
    break;

  case 10:

/* Line 678 of lalr1.cc  */
#line 160 "parser.yy"
    {
         (yyval.commandNode) = new AddNode(*(yysemantic_stack_[(9) - (2)].stringVal), (yysemantic_stack_[(9) - (3)].integerVal), 6);
               delete (yysemantic_stack_[(9) - (2)].stringVal);
     }
    break;

  case 11:

/* Line 678 of lalr1.cc  */
#line 165 "parser.yy"
    {
              std::vector<double> values;
              values.push_back((yysemantic_stack_[(32) - (6)].doubleVal));
              values.push_back((yysemantic_stack_[(32) - (7)].doubleVal));
              values.push_back((yysemantic_stack_[(32) - (8)].doubleVal));
              values.push_back((yysemantic_stack_[(32) - (9)].doubleVal));
              values.push_back((yysemantic_stack_[(32) - (10)].doubleVal));
              values.push_back((yysemantic_stack_[(32) - (11)].doubleVal));
              std::vector<double> information;
              information.push_back((yysemantic_stack_[(32) - (12)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (13)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (14)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (15)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (16)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (17)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (18)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (19)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (20)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (21)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (22)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (23)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (24)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (25)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (26)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (27)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (28)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (29)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (30)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (31)].doubleVal));
              information.push_back((yysemantic_stack_[(32) - (32)].doubleVal));
              (yyval.commandNode) = new AddEdge(*(yysemantic_stack_[(32) - (2)].stringVal), (yysemantic_stack_[(32) - (3)].integerVal), values.size(), (yysemantic_stack_[(32) - (4)].integerVal), (yysemantic_stack_[(32) - (5)].integerVal), values, information);
              delete (yysemantic_stack_[(32) - (2)].stringVal);
     }
    break;

  case 12:

/* Line 678 of lalr1.cc  */
#line 200 "parser.yy"
    {
              std::vector<int> values;
              values.push_back((yysemantic_stack_[(2) - (2)].integerVal));
              (yyval.commandNode) = new FixNode("FIX", values);
           }
    break;

  case 13:

/* Line 678 of lalr1.cc  */
#line 207 "parser.yy"
    {
              (yyval.commandNode) = new SolveSate("SOLVE_STATE");
            }
    break;

  case 14:

/* Line 678 of lalr1.cc  */
#line 212 "parser.yy"
    {
              (yyval.commandNode) = new QueryState("QUERY_STATE");
            }
    break;

  case 15:

/* Line 678 of lalr1.cc  */
#line 216 "parser.yy"
    {
              (yyval.commandNode) = new QueryState("QUERY_STATE", *(yysemantic_stack_[(2) - (2)].intList));
              delete (yysemantic_stack_[(2) - (2)].intList);
            }
    break;

  case 16:

/* Line 678 of lalr1.cc  */
#line 222 "parser.yy"
    {
          (yyval.commandNode) = (yysemantic_stack_[(1) - (1)].commandNode);
        }
    break;

  case 17:

/* Line 678 of lalr1.cc  */
#line 226 "parser.yy"
    {
          (yyval.commandNode) = (yysemantic_stack_[(1) - (1)].commandNode);
        }
    break;

  case 18:

/* Line 678 of lalr1.cc  */
#line 230 "parser.yy"
    {
          (yyval.commandNode) = (yysemantic_stack_[(1) - (1)].commandNode);
        }
    break;

  case 19:

/* Line 678 of lalr1.cc  */
#line 234 "parser.yy"
    {
          (yyval.commandNode) = (yysemantic_stack_[(1) - (1)].commandNode);
        }
    break;

  case 20:

/* Line 678 of lalr1.cc  */
#line 238 "parser.yy"
    {
          (yyval.commandNode) = (yysemantic_stack_[(1) - (1)].commandNode);
        }
    break;

  case 23:

/* Line 678 of lalr1.cc  */
#line 245 "parser.yy"
    {
            driver.slamContext.process((yysemantic_stack_[(3) - (2)].commandNode));
            delete (yysemantic_stack_[(3) - (2)].commandNode);
    }
    break;



/* Line 678 of lalr1.cc  */
#line 749 "bison_parser.cpp"
  default:
          break;
      }
    YY_SYMBOL_PRINT ("-> $$ =", yyr1_[yyn], &yyval, &yyloc);

    yypop_ (yylen);
    yylen = 0;
    YY_STACK_PRINT ();

    yysemantic_stack_.push (yyval);
    yylocation_stack_.push (yyloc);

    /* Shift the result of the reduction.  */
    yyn = yyr1_[yyn];
    yystate = yypgoto_[yyn - yyntokens_] + yystate_stack_[0];
    if (0 <= yystate && yystate <= yylast_
  && yycheck_[yystate] == yystate_stack_[0])
      yystate = yytable_[yystate];
    else
      yystate = yydefgoto_[yyn - yyntokens_];
    goto yynewstate;

  /*------------------------------------.
  | yyerrlab -- here on detecting error |
  `------------------------------------*/
  yyerrlab:
    /* If not already recovering from an error, report this error.  */
    if (!yyerrstatus_)
      {
  ++yynerrs_;
  error (yylloc, yysyntax_error_ (yystate, yytoken));
      }

    yyerror_range[0] = yylloc;
    if (yyerrstatus_ == 3)
      {
  /* If just tried and failed to reuse lookahead token after an
   error, discard it.  */

  if (yychar <= yyeof_)
    {
    /* Return failure if at end of input.  */
    if (yychar == yyeof_)
      YYABORT;
    }
  else
    {
      yydestruct_ ("Error: discarding", yytoken, &yylval, &yylloc);
      yychar = yyempty_;
    }
      }

    /* Else will try to reuse lookahead token after shifting the error
       token.  */
    goto yyerrlab1;


  /*---------------------------------------------------.
  | yyerrorlab -- error raised explicitly by YYERROR.  |
  `---------------------------------------------------*/
  yyerrorlab:

    /* Pacify compilers like GCC when the user code never invokes
       YYERROR and the label yyerrorlab therefore never appears in user
       code.  */
    if (false)
      goto yyerrorlab;

    yyerror_range[0] = yylocation_stack_[yylen - 1];
    /* Do not reclaim the symbols of the rule which action triggered
       this YYERROR.  */
    yypop_ (yylen);
    yylen = 0;
    yystate = yystate_stack_[0];
    goto yyerrlab1;

  /*-------------------------------------------------------------.
  | yyerrlab1 -- common code for both syntax error and YYERROR.  |
  `-------------------------------------------------------------*/
  yyerrlab1:
    yyerrstatus_ = 3;  /* Each real token shifted decrements this.  */

    for (;;)
      {
  yyn = yypact_[yystate];
  if (yyn != yypact_ninf_)
  {
    yyn += yyterror_;
    if (0 <= yyn && yyn <= yylast_ && yycheck_[yyn] == yyterror_)
      {
        yyn = yytable_[yyn];
        if (0 < yyn)
    break;
      }
  }

  /* Pop the current state because it cannot handle the error token.  */
  if (yystate_stack_.height () == 1)
  YYABORT;

  yyerror_range[0] = yylocation_stack_[0];
  yydestruct_ ("Error: popping",
         yystos_[yystate],
         &yysemantic_stack_[0], &yylocation_stack_[0]);
  yypop_ ();
  yystate = yystate_stack_[0];
  YY_STACK_PRINT ();
      }

    yyerror_range[1] = yylloc;
    // Using YYLLOC is tempting, but would change the location of
    // the lookahead.  YYLOC is available though.
    YYLLOC_DEFAULT (yyloc, (yyerror_range - 1), 2);
    yysemantic_stack_.push (yylval);
    yylocation_stack_.push (yyloc);

    /* Shift the error token.  */
    YY_SYMBOL_PRINT ("Shifting", yystos_[yyn],
         &yysemantic_stack_[0], &yylocation_stack_[0]);

    yystate = yyn;
    goto yynewstate;

    /* Accept.  */
  yyacceptlab:
    yyresult = 0;
    goto yyreturn;

    /* Abort.  */
  yyabortlab:
    yyresult = 1;
    goto yyreturn;

  yyreturn:
    if (yychar != yyempty_)
      yydestruct_ ("Cleanup: discarding lookahead", yytoken, &yylval, &yylloc);

    /* Do not reclaim the symbols of the rule which action triggered
       this YYABORT or YYACCEPT.  */
    yypop_ (yylen);
    while (yystate_stack_.height () != 1)
      {
  yydestruct_ ("Cleanup: popping",
       yystos_[yystate_stack_[0]],
       &yysemantic_stack_[0],
       &yylocation_stack_[0]);
  yypop_ ();
      }

    return yyresult;
  }

  // Generate an error message.
  std::string
  Parser::yysyntax_error_ (int yystate, int tok)
  {
    std::string res;
    YYUSE (yystate);
#if YYERROR_VERBOSE
    int yyn = yypact_[yystate];
    if (yypact_ninf_ < yyn && yyn <= yylast_)
      {
  /* Start YYX at -YYN if negative to avoid negative indexes in
     YYCHECK.  */
  int yyxbegin = yyn < 0 ? -yyn : 0;

  /* Stay within bounds of both yycheck and yytname.  */
  int yychecklim = yylast_ - yyn + 1;
  int yyxend = yychecklim < yyntokens_ ? yychecklim : yyntokens_;
  int count = 0;
  for (int x = yyxbegin; x < yyxend; ++x)
    if (yycheck_[x + yyn] == x && x != yyterror_)
      ++count;

  // FIXME: This method of building the message is not compatible
  // with internationalization.  It should work like yacc.c does it.
  // That is, first build a string that looks like this:
  // "syntax error, unexpected %s or %s or %s"
  // Then, invoke YY_ on this string.
  // Finally, use the string as a format to output
  // yytname_[tok], etc.
  // Until this gets fixed, this message appears in English only.
  res = "syntax error, unexpected ";
  res += yytnamerr_ (yytname_[tok]);
  if (count < 5)
    {
      count = 0;
      for (int x = yyxbegin; x < yyxend; ++x)
        if (yycheck_[x + yyn] == x && x != yyterror_)
    {
      res += (!count++) ? ", expecting " : " or ";
      res += yytnamerr_ (yytname_[x]);
    }
    }
      }
    else
#endif
      res = YY_("syntax error");
    return res;
  }


  /* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
     STATE-NUM.  */
  const signed char Parser::yypact_ninf_ = -24;
  const signed char
  Parser::yypact_[] =
  {
       -24,    54,   -24,   -24,    -7,     1,   -24,    10,   -24,   -24,
     -24,   -24,   -24,     9,    31,    51,    52,    55,   -24,   -24,
      56,   -24,     4,     4,    58,    59,   -24,   -24,   -24,     4,
       4,    60,    61,     4,     4,     4,     4,   -24,     4,     4,
       4,     4,     4,     4,     4,     4,     4,   -24,     4,     4,
       4,     4,     4,     4,     4,     4,     4,     4,   -24,     4,
       4,     4,     4,     4,     4,     4,     4,     4,     4,     4,
       4,     4,     4,     4,     4,     4,     4,   -24
  };

  /* YYDEFACT[S] -- default rule to reduce with in state S when YYTABLE
     doesn't specify something else to do.  Zero means the default is an
     error.  */
  const unsigned char
  Parser::yydefact_[] =
  {
        21,     0,     1,    22,     0,     0,    13,    14,    16,    17,
      18,    19,    20,     0,     0,     0,     0,     0,    12,     2,
      15,    23,     6,     9,     0,     0,     3,     4,     5,     0,
       0,     0,     0,     0,     0,     0,     0,     7,     0,     0,
       0,     0,     0,     0,     0,     0,     0,    10,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     8,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,    11
  };

  /* YYPGOTO[NTERM-NUM].  */
  const signed char
  Parser::yypgoto_[] =
  {
       -24,   -24,   -23,   -24,   -24,   -24,   -24,   -24,   -24,   -24
  };

  /* YYDEFGOTO[NTERM-NUM].  */
  const signed char
  Parser::yydefgoto_[] =
  {
        -1,    20,    29,     8,     9,    10,    11,    12,    13,     1
  };

  /* YYTABLE[YYPACT[STATE-NUM]].  What to do in state STATE-NUM.  If
     positive, shift that token.  If negative, reduce the rule which
     number is the opposite.  If zero, do what YYDEFACT says.  */
  const signed char Parser::yytable_ninf_ = -1;
  const unsigned char
  Parser::yytable_[] =
  {
        30,    14,    15,    16,    17,    18,    33,    34,    27,    28,
      37,    38,    39,    40,    19,    41,    42,    43,    44,    45,
      46,    47,    48,    49,    21,    50,    51,    52,    53,    54,
      55,    56,    57,    58,    59,    22,    60,    61,    62,    63,
      64,    65,    66,    67,    68,    69,    70,    71,    72,    73,
      74,    75,    76,    77,     2,    23,    24,     3,     0,    25,
      26,     4,    31,    32,    35,    36,     5,     6,     7
  };

  /* YYCHECK.  */
  const signed char
  Parser::yycheck_[] =
  {
        23,     8,     9,    10,    11,     4,    29,    30,     4,     5,
      33,    34,    35,    36,     4,    38,    39,    40,    41,    42,
      43,    44,    45,    46,    15,    48,    49,    50,    51,    52,
      53,    54,    55,    56,    57,     4,    59,    60,    61,    62,
      63,    64,    65,    66,    67,    68,    69,    70,    71,    72,
      73,    74,    75,    76,     0,     4,     4,     3,    -1,     4,
       4,     7,     4,     4,     4,     4,    12,    13,    14
  };

  /* STOS_[STATE-NUM] -- The (internal number of the) accessing
     symbol of state STATE-NUM.  */
  const unsigned char
  Parser::yystos_[] =
  {
         0,    25,     0,     3,     7,    12,    13,    14,    19,    20,
      21,    22,    23,    24,     8,     9,    10,    11,     4,     4,
      17,    15,     4,     4,     4,     4,     4,     4,     5,    18,
      18,     4,     4,    18,    18,     4,     4,    18,    18,    18,
      18,    18,    18,    18,    18,    18,    18,    18,    18,    18,
      18,    18,    18,    18,    18,    18,    18,    18,    18,    18,
      18,    18,    18,    18,    18,    18,    18,    18,    18,    18,
      18,    18,    18,    18,    18,    18,    18,    18
  };

#if YYDEBUG
  /* TOKEN_NUMBER_[YYLEX-NUM] -- Internal symbol number corresponding
     to YYLEX-NUM.  */
  const unsigned short int
  Parser::yytoken_number_[] =
  {
         0,   256,   257,   258,   259,   260,   261,   262,   263,   264,
     265,   266,   267,   268,   269,    59
  };
#endif

  /* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
  const unsigned char
  Parser::yyr1_[] =
  {
         0,    16,    17,    17,    18,    18,    19,    19,    19,    20,
      20,    20,    21,    22,    23,    23,    24,    24,    24,    24,
      24,    25,    25,    25
  };

  /* YYR2[YYN] -- Number of symbols composing right hand side of rule YYN.  */
  const unsigned char
  Parser::yyr2_[] =
  {
         0,     2,     1,     2,     1,     1,     3,     6,    14,     3,
       9,    32,     2,     1,     1,     2,     1,     1,     1,     1,
       1,     0,     2,     3
  };

#if YYDEBUG || YYERROR_VERBOSE || YYTOKEN_TABLE
  /* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
     First, the terminals, then, starting at \a yyntokens_, nonterminals.  */
  const char*
  const Parser::yytname_[] =
  {
    "\"end of file\"", "error", "$undefined", "\"end of line\"",
  "\"integer\"", "\"double\"", "\"string\"", "\"ADD\"", "\"Vertex SE2\"",
  "\"Vertex SE3\"", "\"Edge SE2\"", "\"Edge SE3\"", "\"Fix\"",
  "\"Solve State\"", "\"Query State\"", "';'", "$accept", "int_list",
  "NUMBER", "add_se2", "add_se3", "fix_node", "solve_state", "query_state",
  "command", "start", 0
  };
#endif

#if YYDEBUG
  /* YYRHS -- A `-1'-separated list of the rules' RHS.  */
  const Parser::rhs_number_type
  Parser::yyrhs_[] =
  {
        25,     0,    -1,     4,    -1,    17,     4,    -1,     4,    -1,
       5,    -1,     7,     8,     4,    -1,     7,     8,     4,    18,
      18,    18,    -1,     7,    10,     4,     4,     4,    18,    18,
      18,    18,    18,    18,    18,    18,    18,    -1,     7,     9,
       4,    -1,     7,     9,     4,    18,    18,    18,    18,    18,
      18,    -1,     7,    11,     4,     4,     4,    18,    18,    18,
      18,    18,    18,    18,    18,    18,    18,    18,    18,    18,
      18,    18,    18,    18,    18,    18,    18,    18,    18,    18,
      18,    18,    18,    18,    -1,    12,     4,    -1,    13,    -1,
      14,    -1,    14,    17,    -1,    19,    -1,    20,    -1,    21,
      -1,    22,    -1,    23,    -1,    -1,    25,     3,    -1,    25,
      24,    15,    -1
  };

  /* YYPRHS[YYN] -- Index of the first RHS symbol of rule number YYN in
     YYRHS.  */
  const unsigned char
  Parser::yyprhs_[] =
  {
         0,     0,     3,     5,     8,    10,    12,    16,    23,    38,
      42,    52,    85,    88,    90,    92,    95,    97,    99,   101,
     103,   105,   106,   109
  };

  /* YYRLINE[YYN] -- Source line where rule number YYN was defined.  */
  const unsigned char
  Parser::yyrline_[] =
  {
         0,   103,   103,   108,   114,   118,   123,   128,   137,   154,
     159,   164,   199,   206,   211,   215,   221,   225,   229,   233,
     237,   242,   243,   244
  };

  // Print the state stack on the debug stream.
  void
  Parser::yystack_print_ ()
  {
    *yycdebug_ << "Stack now";
    for (state_stack_type::const_iterator i = yystate_stack_.begin ();
   i != yystate_stack_.end (); ++i)
      *yycdebug_ << ' ' << *i;
    *yycdebug_ << std::endl;
  }

  // Report on the debug stream that the rule \a yyrule is going to be reduced.
  void
  Parser::yy_reduce_print_ (int yyrule)
  {
    unsigned int yylno = yyrline_[yyrule];
    int yynrhs = yyr2_[yyrule];
    /* Print the symbols being reduced, and their result.  */
    *yycdebug_ << "Reducing stack by rule " << yyrule - 1
         << " (line " << yylno << "):" << std::endl;
    /* The symbols being reduced.  */
    for (int yyi = 0; yyi < yynrhs; yyi++)
      YY_SYMBOL_PRINT ("   $" << yyi + 1 << " =",
           yyrhs_[yyprhs_[yyrule] + yyi],
           &(yysemantic_stack_[(yynrhs) - (yyi + 1)]),
           &(yylocation_stack_[(yynrhs) - (yyi + 1)]));
  }
#endif // YYDEBUG

  /* YYTRANSLATE(YYLEX) -- Bison symbol number corresponding to YYLEX.  */
  Parser::token_number_type
  Parser::yytranslate_ (int t)
  {
    static
    const token_number_type
    translate_table[] =
    {
           0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,    15,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14
    };
    if ((unsigned int) t <= yyuser_token_number_max_)
      return translate_table[t];
    else
      return yyundef_token_;
  }

  const int Parser::yyeof_ = 0;
  const int Parser::yylast_ = 68;
  const int Parser::yynnts_ = 10;
  const int Parser::yyempty_ = -2;
  const int Parser::yyfinal_ = 2;
  const int Parser::yyterror_ = 1;
  const int Parser::yyerrcode_ = 256;
  const int Parser::yyntokens_ = 16;

  const unsigned int Parser::yyuser_token_number_max_ = 269;
  const Parser::token_number_type Parser::yyundef_token_ = 2;


/* Line 1054 of lalr1.cc  */
#line 1 "[Bison:b4_percent_define_default]"

} // SlamParser

/* Line 1054 of lalr1.cc  */
#line 1214 "bison_parser.cpp"


/* Line 1056 of lalr1.cc  */
#line 252 "parser.yy"
 /*** Additional Code ***/

void SlamParser::Parser::error(const Parser::location_type& l, const std::string& m)
{
  driver.error(l, m);
}

