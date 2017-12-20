/* Copyright (c) 2017, Benjamin Shropshire,
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

%option reentrant bison-bridge yylineno bison-locations noinput nounput
%{
// lexer.yy.cc can't allow include lexer.yy.h so pretend it's already happened
#define yyHEADER_H

#include "stl-to-ps/ast.h"
#include "stl-to-ps/gen.lexer.h"
#include "stl-to-ps/gen.parser.h"
#include "stl-to-ps/parser_support.h"

typedef yy::parser::token T;

int yyget_column(yyFlexLexer* yyscanner);

#define YY_USER_ACTION do {                         \
    auto* loc = yyget_lloc(yyscanner);              \
    int line = yyget_lineno(yyscanner);             \
    int col = yyget_column(yyscanner);              \
    auto fn = (std::string*)yyget_extra(yyscanner); \
    loc->begin.filename = loc->end.filename = fn;   \
    loc->begin.line = loc->end.line = line;         \
    loc->begin.column = col;                        \
    loc->end.column = col + yyleng;                 \
    yyset_column(col + yyleng, yyscanner);          \
} while(0);

extern "C" int yywrap(yyscan_t yyscanner) { return 1; }

void yyerror(yyFlexLexer* scanner) {
  yy::parser::location_type *yyget_lloc (yyscan_t yyscanner );
  char *yyget_text (yyscan_t yyscanner );

  auto &loc = *yyget_lloc(scanner);
  stl2ps::error(loc.begin.filename,
                loc.begin.line, loc.begin.column, loc.end.line, loc.end.column,
                yyget_text(scanner));
}

%}

%%
\{  return '{';
\}  return '}';
\[  return '[';
\]  return ']';
\(  return '(';
\)  return ')';
=   return '=';
;   return ';';
:   return ':';
,   return ',';
@   return '@';

angle return T::ANGLE;
dim   return T::DIM;
draw  return T::DRAW;
load  return T::LOAD;
page  return T::PAGE;
text  return T::TEXT;

[a-zA-Z_][a-zA-Z0-9_]*      { yylval_param->str = new std::string(yytext);  return T::ID; }
[-+]?[0-9]+(\.[0-9]*)?      { yylval_param->fp  = std::stof(yytext);        return T::NUM; }
[-+]?\.[0-9]+               { yylval_param->fp  = std::stof(yytext);        return T::NUM; }
\"([^"\\\n]|\\['"?\\n])*\"  { yylval_param->str = stl2ps::NewQuote(yytext); return T::STRING_LIT; }

[ \t\n\r]    ;
\/\/.*      ;

.      { yyerror(yyscanner); return 256; }
%%