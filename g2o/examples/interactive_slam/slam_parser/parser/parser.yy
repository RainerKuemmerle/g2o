%{ /*** C/C++ Declarations ***/

#include <stdio.h>
#include <string>
#include <vector>

#include "commands.h"

%}

/*** yacc/bison Declarations ***/

/* Require bison 2.3 or later */
%require "2.3"

/* add debug output code to generated parser. disable this for release
 * versions. */
/*%debug*/

/* start symbol is named "start" */
%start start

/* write out a header file containing the token defines */
%defines

/* use newer C++ skeleton file */
%skeleton "lalr1.cc"

/* namespace to enclose parser in */
%name-prefix="SlamParser"

/* set the parser's class identifier */
%define "parser_class_name" "Parser"

/* keep track of the current position within the input */
%locations
%initial-action
{
    // initialize the initial location object
    @$.begin.filename = @$.end.filename = &driver.streamname;
};

/* The driver is passed by reference to the parser and to the scanner. This
 * provides a simple but effective pure interface, not relying on global
 * variables. */
%parse-param { class Driver& driver }

/* verbose error messages */
%error-verbose

 /*** BEGIN TOKEN - Change the example grammar's tokens below ***/

%union {
    int  			integerVal;
    double 			doubleVal;
    std::string*		stringVal;
    class CommandNode*		commandNode;
    std::vector<int>*           intList;
}

%token			END	     0	"end of file"
%token			EOL		"end of line"
%token <integerVal> 	INTEGER		"integer"
%token <doubleVal> 	DOUBLE		"double"
%token <stringVal> 	STRING		"string"
%token <stringVal> 	ADD		"ADD"
%token <stringVal> 	V_SE2		"Vertex SE2"
%token <stringVal> 	V_SE3		"Vertex SE3"
%token <stringVal> 	E_SE2		"Edge SE2"
%token <stringVal> 	E_SE3		"Edge SE3"
%token <stringVal> 	FIX		"Fix"
%token <stringVal> 	SOLVE_STATE	"Solve State"
%token <stringVal> 	QUERY_STATE	"Query State"

%type <doubleVal>	NUMBER
%type <commandNode>	query_state solve_state fix_node add_se2 add_se3 command
%type <intList>	        int_list

%destructor { delete $$; } STRING
%destructor { delete $$; } solve_state query_state fix_node add_se2 add_se3 command
/* %destructor { delete $$; } int_list */

 /*** END TOKEN - Change the example grammar's tokens above ***/

%{

#include "driver.h"
#include "scanner.h"
#include "slam_context.h"

/* this "connects" the bison parser in the driver to the flex scanner class
 * object. it defines the yylex() function call to pull the next token from the
 * current lexer object of the driver context. */
#undef yylex
#define yylex driver.lexer->lex

%}

%% /*** Grammar Rules ***/

 /*** BEGIN GRAMMAR ***/

int_list : INTEGER
           {
              $$ = new std::vector<int>;
              $$->push_back($1);
           }
         | int_list INTEGER
           {
             $1->push_back($2);
             $$ = $1; 
           }

NUMBER : INTEGER
         {
	   $$ = $1;
	 }
       | DOUBLE
         {
	   $$ = $1;
	 }

add_se2 : ADD V_SE2 INTEGER
           {
	       $$ = new AddNode(*$2, $3, 3);
               delete $2;
	   }
         | ADD V_SE2 INTEGER NUMBER NUMBER NUMBER
           {
              std::vector<double> values;
              values.push_back($4);
              values.push_back($5);
              values.push_back($6);
	      $$ = new AddNode(*$2, $3, 3, values);
              delete $2;
	   }
         | ADD E_SE2 INTEGER INTEGER INTEGER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER   
           {
              std::vector<double> values;
              values.push_back($6);
              values.push_back($7);
              values.push_back($8);
              std::vector<double> information;
              information.push_back($9);
              information.push_back($10);
              information.push_back($11);
              information.push_back($12);
              information.push_back($13);
              information.push_back($14);
              $$ = new AddEdge(*$2, $3, values.size(), $4, $5, values, information);
              delete $2;
	   }

add_se3 : ADD V_SE3 INTEGER
           {
               $$ = new AddNode(*$2, $3, 6);
               delete $2;
	   }
         | ADD V_SE3 INTEGER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER
           {
	       $$ = new AddNode(*$2, $3, 6);
               delete $2;
	   }
         | ADD E_SE3 INTEGER INTEGER INTEGER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER NUMBER
           {
              std::vector<double> values;
              values.push_back($6);
              values.push_back($7);
              values.push_back($8);
              values.push_back($9);
              values.push_back($10);
              values.push_back($11);
              std::vector<double> information;
              information.push_back($12);
              information.push_back($13);
              information.push_back($14);
              information.push_back($15);
              information.push_back($16);
              information.push_back($17);
              information.push_back($18);
              information.push_back($19);
              information.push_back($20);
              information.push_back($21);
              information.push_back($22);
              information.push_back($23);
              information.push_back($24);
              information.push_back($25);
              information.push_back($26);
              information.push_back($27);
              information.push_back($28);
              information.push_back($29);
              information.push_back($30);
              information.push_back($31);
              information.push_back($32);
              $$ = new AddEdge(*$2, $3, values.size(), $4, $5, values, information);
              delete $2;
	   }

fix_node : FIX INTEGER
           {
              std::vector<int> values;
              values.push_back($2);
              $$ = new FixNode("FIX", values);
           }

solve_state : SOLVE_STATE
            {
              $$ = new SolveSate("SOLVE_STATE");
            }

query_state : QUERY_STATE 
            {
              $$ = new QueryState("QUERY_STATE");
            }
            | QUERY_STATE int_list
            {
              $$ = new QueryState("QUERY_STATE", *$2);
              delete $2;
            }

command : add_se2
        {
          $$ = $1;
        }
        | add_se3
        {
          $$ = $1;
        }
        | fix_node
        {
          $$ = $1;
        }
        | solve_state
        {
          $$ = $1;
        }
        | query_state
        {
          $$ = $1;
        }

start	: /* empty */
        | start EOL
        | start command ';'
          {
            driver.slamContext.process($2);
            delete $2;
	  }

 /*** END GRAMMAR ***/

%% /*** Additional Code ***/

void SlamParser::Parser::error(const Parser::location_type& l, const std::string& m)
{
  driver.error(l, m);
}
