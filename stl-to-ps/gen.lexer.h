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

#ifndef STL_TO_PS_GEN_LEXER_H_
#define STL_TO_PS_GEN_LEXER_H_

// Proxy header for the lexer.

#include "stl-to-ps/ast.h"

typedef void yyFlexLexer;

#include "stl-to-ps/parser.tab.h"

typedef yy::parser::semantic_type YYSTYPE;
typedef yy::parser::location_type YYLTYPE;

int yylex(yy::parser::semantic_type*, yy::parser::location_type*,
          yyFlexLexer* yyscanner);

// Hack to make this work for the first line.
void yyset_column(int column_no , yyFlexLexer* yyscanner);

// So that we can make yylex be a wrapper.
#define YY_DECL int yylex_innner(                \
      yy::parser::semantic_type * yylval_param,  \
      yy::parser::location_type * yylloc_param,  \
      yyscan_t yyscanner)

#include "stl-to-ps/lexer.yy.h"

#endif  // STL_TO_PS_GEN_LEXER_H_
