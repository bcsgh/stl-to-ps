// Copyright (c) 2018, Benjamin Shropshire,
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

#include <cstdlib>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "gflags/gflags.h"  // TODO dump
#include "glog/logging.h"
#include "gtest/gtest.h"

ABSL_FLAG(int32_t, test_srand_seed, 0, "The seed used for random");

// TODO: Dump this once absl get logging.
ABSL_FLAG(bool, alsologtostderr_x, false,
          "log messages go to stderr in addition to logfiles");
ABSL_FLAG(bool, logtostderr_x, false,
          "log messages go to stderr instead of logfiles");
ABSL_FLAG(int32_t, v_x, 0,  //
          "Show all VLOG(m) messages for m <= this.");

DECLARE_bool(alsologtostderr);
DECLARE_bool(logtostderr);
DECLARE_int32(v);

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  auto args = absl::ParseCommandLine(argc, argv);
  // Forward flags to glog (it doesn't use absl::Flags).
  FLAGS_alsologtostderr = absl::GetFlag(FLAGS_alsologtostderr_x);
  FLAGS_logtostderr = absl::GetFlag(FLAGS_logtostderr_x);
  FLAGS_v = absl::GetFlag(FLAGS_v_x);
  google::InitGoogleLogging(args[0]);
  google::InstallFailureSignalHandler();

  int32_t test_srand_seed = absl::GetFlag(FLAGS_test_srand_seed);
  if (test_srand_seed == 0) {
    test_srand_seed = absl::ToUnixMicros(absl::Now());
  }
  LOG(WARNING) << "--test_srand_seed=" << test_srand_seed;
  std::srand(test_srand_seed);

  return RUN_ALL_TESTS();
}