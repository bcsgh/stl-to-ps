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

#include "stl-to-ps/parser.h"

#include "absl/flags/flag.h"
#include "parser/parser_support.h"
#include "stl-to-ps/ast.h"
#include "stl-to-ps/gen.lexer.h"

#if defined(YYDEBUG) && YYDEBUG
ABSL_FLAG(bool, parser_debug, false, "Enable debuging of the parser");
#endif

YY_DECL;  // Forward declare
int stl2pslex(stl2ps_parser::parser::semantic_type* stype,
              stl2ps_parser::parser::location_type* loc, stl2psscan_t scanner) {
  return stl2pslex_inner(stype, loc, scanner);
}

namespace stl2ps {

int Parse(std::string filename, const std::string& file, Document* doc) {
  parser_support::ScannerExtra extra;
  extra.filename = &filename;

  stl2psscan_t scanner;
  stl2pslex_init(&scanner);
  auto buffer_state = stl2ps_scan_bytes(file.data(), file.size(), scanner);
  stl2psset_lineno(1, scanner);
  stl2psset_column(0, scanner);
  stl2psset_extra(&extra, scanner);
  int ret;
  {
    stl2ps_parser::parser p{scanner, doc, &extra};
#if defined(YYDEBUG) && YYDEBUG
    p.set_debug_level(absl::GetFlag(FLAGS_parser_debug));
    p.set_debug_stream(std::cout);
#endif
    ret = p.parse();
  }
  stl2ps_delete_buffer(buffer_state, scanner);
  stl2pslex_destroy(scanner);
  return ret;
}

}  // namespace stl2ps
