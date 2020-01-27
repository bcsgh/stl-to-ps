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

#include "stl-to-ps/stl-to-ps.h"

#include <fstream>
#include <iostream>
#include <string>

#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(script, "", "The stl file to read from");
DEFINE_string(output, "", "The ps file to write to");

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // LOG(INFO) << FLAGS_input << " -> " << FLAGS_output;
  if (FLAGS_script.empty()) {
    LOG(INFO) << "No script file";
    return 1;
  }
  if (FLAGS_output.empty()) {
    LOG(INFO) << "No output file";
    return 1;
  }
  LOG(INFO) << FLAGS_script << " -> " << FLAGS_output;

  std::ofstream out;
  out.open(FLAGS_output, (std::ios::out | std::ios::trunc));
  CHECK(out.rdstate() == std::ios_base::goodbit) << FLAGS_output;

  CHECK(!FLAGS_script.empty());
  std::ifstream in;
  in.open(FLAGS_script, std::ios::in);
  CHECK(in.rdstate() == std::ios_base::goodbit) << FLAGS_script;

  bool ret = stl2ps::ScriptToPS(FLAGS_script, in, out);

  in.close();
  out.close();

  return ret ? 0 : 1;
}
