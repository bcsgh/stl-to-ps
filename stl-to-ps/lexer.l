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
%option prefix="stl2ps"
%{
// lexer.yy.cc can't allow include lexer.yy.h so pretend it's already happened
#define stl2psHEADER_H

#include "parser/parser_support.h"
#include "stl-to-ps/ast.h"
#include "stl-to-ps/gen.lexer.h"
#include "stl-to-ps/gen.parser.h"

typedef stl2ps_parser::parser::token T;

int stl2psget_column(stl2psscan_t scanner);

#define YY_USER_ACTION do {                          \
    auto* loc = stl2psget_lloc(yyscanner);           \
    int line = stl2psget_lineno(yyscanner);          \
    int col = stl2psget_column(yyscanner);           \
    auto e = (parser_support::ScannerExtra*)         \
        stl2psget_extra(yyscanner);                  \
    loc->begin.filename = loc->end.filename =        \
        e->filename;                                 \
    loc->begin.line = loc->end.line = line;          \
    loc->begin.column = col;                         \
    loc->end.column = col + yyleng;                  \
    stl2psset_column(col + yyleng, yyscanner);       \
} while(0);

extern "C" int stl2pswrap(stl2psscan_t scanner) { return 1; }

void stl2pserror(stl2psscan_t scanner) {
  // Declare some functions defined in the generated code.
  stl2ps_parser::parser::location_type *stl2psget_lloc(stl2psscan_t scanner);
  void *stl2psget_extra(stl2psscan_t scanner);
  char *stl2psget_text(stl2psscan_t scanner);

  auto &loc = *stl2psget_lloc(scanner);
  auto e = (parser_support::ScannerExtra*)stl2psget_extra(scanner);
  parser_support::Error(loc, stl2psget_text(scanner), e);
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
dia   return T::DIA;
dim   return T::DIM;
draw  return T::DRAW;
load  return T::LOAD;
page  return T::PAGE;
rad   return T::RAD;
text  return T::TEXT;

[a-zA-Z_][a-zA-Z0-9_]*      { yylval_param->str = new std::string(yytext);  return T::ID; }
[-+]?[0-9]+(\.[0-9]*)?      { yylval_param->fp  = std::stof(yytext);        return T::NUM; }
[-+]?\.[0-9]+               { yylval_param->fp  = std::stof(yytext);        return T::NUM; }
\"([^"\\\n]|\\['"?\\n])*\"  { yylval_param->str = stl2ps::NewQuote(yytext); return T::STRING_LIT; }

[ \t\n\r]    ;
\/\/.*      ;

.      { stl2pserror(yyscanner); return 256; }
%%