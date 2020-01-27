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

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "gflags/gflags.h"  // TODO dump
#include "glog/logging.h"

ABSL_FLAG(std::string, script, "", "The stl file to read from");
ABSL_FLAG(std::string, output, "", "The ps file to write to");

// TODO: Dump this once absl get logging.
ABSL_FLAG(bool, alsologtostderr_x, false,
          "log messages go to stderr in addition to logfiles");
ABSL_FLAG(bool, logtostderr_x, false,
          "log messages go to stderr instead of logfiles");
ABSL_FLAG(int32_t, v_x, 0,
          "Show all VLOG(m) messages for m <= this.");

DECLARE_bool(alsologtostderr);
DECLARE_bool(logtostderr);
DECLARE_int32(v);

int main(int argc, char** argv) {
  auto args = absl::ParseCommandLine(argc, argv);
  // Forward flags to glog (it doesn't use absl::Flags).
  FLAGS_alsologtostderr = absl::GetFlag(FLAGS_alsologtostderr_x);
  FLAGS_logtostderr = absl::GetFlag(FLAGS_logtostderr_x);
  FLAGS_v = absl::GetFlag(FLAGS_v_x);
  google::InitGoogleLogging(args[0]);

  if (absl::GetFlag(FLAGS_script).empty()) {
    LOG(INFO) << "No script file";
    return 1;
  }
  if (absl::GetFlag(FLAGS_output).empty()) {
    LOG(INFO) << "No output file";
    return 1;
  }
  LOG(INFO) << absl::GetFlag(FLAGS_script) << " -> "
            << absl::GetFlag(FLAGS_output);

  std::ofstream out;
  out.open(absl::GetFlag(FLAGS_output), (std::ios::out | std::ios::trunc));
  CHECK(out.rdstate() == std::ios_base::goodbit) << absl::GetFlag(FLAGS_output);

  std::ifstream in;
  in.open(absl::GetFlag(FLAGS_script), std::ios::in);
  CHECK(in.rdstate() == std::ios_base::goodbit) << absl::GetFlag(FLAGS_script);

  bool ret = stl2ps::ScriptToPS(absl::GetFlag(FLAGS_script), in, out);

  in.close();
  out.close();

  return ret ? 0 : 1;
}
