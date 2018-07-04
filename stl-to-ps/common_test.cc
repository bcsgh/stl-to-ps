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

#include "stl-to-ps/common.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using testing::ElementsAre;

class LogTesting : public logging::LogBase {
 public:
  static void TestDumpStack() { DumpStack(LOG(ERROR)); }
};

TEST(Logging, Whatever) {
  LOG(INFO) << "Foo";

  LogTesting::TestDumpStack();
}

namespace logging {
namespace logging_internal {

void handler(int sig, bool exit);
}  // namespace logging_internal

TEST(Logging, Signals) {
  logging_internal::handler(0, false);
  logging_internal::handler(SIGSEGV, false);
  logging_internal::handler(SIGTERM, false);
}

TEST(Logging, Demangle) {
  EXPECT_EQ(Demangle<int>(), "int");
  EXPECT_EQ(Demangle<LogBase>(), "logging::LogBase");
  EXPECT_EQ(Demangle<std::string>(), "std::string");
}

}  // namespace logging

namespace base {

TEST(Base, PrintF) {
  EXPECT_EQ(PrintF("%.3f %s %s\n", 1.1, std::string("hello"), "world"),
            "1.100 hello world\n");
}

}  // namespace base
