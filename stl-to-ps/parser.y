// Copyright (c) 2017, Benjamin Shropshire,
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

%skeleton "lalr1.cc"
%debug
%locations

%{
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "absl/memory/memory.h"
#include "parser/parser_support.h"
#include "stl-to-ps/ast.h"
#include "stl-to-ps/gen.lexer.h"

namespace stl2ps_parser {
void parser::error(stl2ps_parser::location const& loc, std::string const& msg) {
  parser_support::Error(loc, msg);
}
} // namespace stl2ps_parser

using stl2ps::Meta;
using stl2ps::PointFunc;
using stl2ps::Scale;

namespace {
int stl2ps_parserlex(stl2ps_parser::parser::semantic_type* o,
                     stl2ps_parser::parser::location_type* l, stl2psscan_t s) {
  return stl2pslex(o, l, s);
}
}  // namespace

%}
%name-prefix "stl2ps_parser"
%param {stl2psscan_t scanner}
%parse-param { stl2ps::Document *result }

%token ANGLE DIA DIM DRAW LOAD PAGE RAD TEXT;
%token NUM ID STRING_LIT;
%type <str> ID STRING_LIT;
%type <fp> NUM;

%union {
  stl2ps::Document*   doc;
  stl2ps::MetaList*   meta_list;
  stl2ps::DrawList*   draw;
  Eigen::RowVector2d* location;
  stl2ps::Model*      model;
  stl2ps::Page*       page;
  stl2ps::PageParts*  page_parts;
  stl2ps::Point*      point;


  stl2ps::Meta* meta;

  std::string* str;
  float fp;
}

%type <meta_list> dim_parts;
%type <draw>     draw_parts;
%type <meta_list> text_parts;
%type <meta>     arg;
%type <doc>      input;
%type <location> location;
%type <model>    model;
%type <page>     page;
%type <point>    point;
%type <page_parts> page_parts;
%type <str>      string_lit;

%% /* Grammar rules and actions follow */

%start input;

location : '[' NUM ',' NUM ']' { $$ = new Eigen::RowVector2d{$2, $4}; }
         ;

point : '[' NUM ',' NUM ',' NUM ']' { $$ = new stl2ps::Val({$2, $4, $6}, @1); }
      | ID '(' point ')' { $$ = new PointFunc(absl::WrapUnique($1), absl::WrapUnique($3), @1); }
      ;

input : input model  { ($$ = $1)->Add(absl::WrapUnique($2)); }
      | input page   { ($$ = $1)->Add(absl::WrapUnique($2)); }
      |              { $$ = result; }
      ;

model : LOAD ID '=' STRING_LIT ';'
           { $$ = new stl2ps::Model(absl::WrapUnique($2), absl::WrapUnique($4), @1); }
      ;

page : PAGE '{' page_parts '}'  { $$ = new stl2ps::Page(@1, absl::WrapUnique($3)); }
     ;

string_lit : string_lit STRING_LIT   { *($$ = $1) += *$2; delete $2; }
           | STRING_LIT              { $$ = $1; };

arg : ID '=' string_lit
         { $$ = Meta::New<std::string>($1, $3, @1).release(); }
    | ID '=' point
         { $$ = Meta::New<stl2ps::Point>($1, $3, @1).release(); }
    | ID '=' NUM
         { $$ = Meta::New<float>($1, $3, @1).release(); }
    | ID '=' location
         { $$ = Meta::New<Eigen::RowVector2d>($1, $3, @1).release(); }
    | ID '=' NUM ':' NUM
         { $$ = Meta::New<Scale>($1, Scale{$3, $5}, @1).release(); }
    | '@' location
         { $$ = Meta::New<Eigen::RowVector2d>("@", $2, @1).release(); }
    ;

page_parts : page_parts arg ';'
                { ($$ = $1)->meta.emplace_back($2); }
           | page_parts DRAW ID '{' draw_parts '}'
                { ($$ = $1)->draws.emplace_back(absl::make_unique<stl2ps::Draw>(@3, absl::WrapUnique($3), absl::WrapUnique($5))); }
           | page_parts TEXT '[' string_lit ']' '{' text_parts '}'
                { ($$ = $1)->draws.emplace_back(absl::make_unique<stl2ps::Text>(@2, absl::WrapUnique($4), absl::WrapUnique($7))); }
           |   /* EMPTY */
                { $$ = new stl2ps::PageParts; }
           ;

draw_parts : draw_parts arg ';'
                { ($$ = $1)->meta_list.emplace_back($2); }
           | draw_parts ANGLE '(' ')' ';'
                {
                  ($$ = $1)->dims.emplace_back(
                      absl::make_unique<stl2ps::Angle>(@2));
                }
           | draw_parts ANGLE '(' dim_parts ')' ';'
                {
                  ($$ = $1)->dims.emplace_back(absl::make_unique<stl2ps::Angle>(@2, std::move(*$4)));
                  delete $4;
                }
           | draw_parts DIA '(' ')' ';'
                 {
                   ($$ = $1)->dims.emplace_back(
                      absl::make_unique<stl2ps::Dia>(@2));
                 }
           | draw_parts DIA '(' dim_parts ')' ';'
                {
                  ($$ = $1)->dims.emplace_back(absl::make_unique<stl2ps::Dia>(@2, std::move(*$4)));
                  delete $4;
                }
           | draw_parts DIM '(' ')' ';'
                {
                  ($$ = $1)->dims.emplace_back(
                      absl::make_unique<stl2ps::Dim>(@2));
                }
           | draw_parts DIM '(' dim_parts ')' ';'
                {
                  ($$ = $1)->dims.emplace_back(absl::make_unique<stl2ps::Dim>(@2, std::move(*$4)));
                  delete $4;
                }
           | draw_parts RAD '(' ')' ';'
                {
                  ($$ = $1)->dims.emplace_back(
                      absl::make_unique<stl2ps::Rad>(std::move(@2)));
                }
           | draw_parts RAD '(' dim_parts ')' ';'
                {
                  ($$ = $1)->dims.emplace_back(absl::make_unique<stl2ps::Rad>(@2, std::move(*$4)));
                  delete $4;
                }
           |   /* EMPTY */
                { $$ = new stl2ps::DrawList; }
           ;

text_parts : text_parts arg ';'
                { ($$ = $1)->emplace_back($2); }
           |   /* EMPTY */
                { $$ = new stl2ps::MetaList; }
           ;

dim_parts : dim_parts ',' arg  { ($$ = $1)->emplace_back($3); }
          | arg                { ($$ = new stl2ps::MetaList)->emplace_back($1); }
          ;