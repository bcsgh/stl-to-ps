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

#include <cxxabi.h>
#include <execinfo.h>
#include <signal.h>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <string>

#include "absl/debugging/stacktrace.h"
#include "absl/strings/str_replace.h"
#include "gflags/gflags.h"

DEFINE_bool(logtostderr, false, "");
DEFINE_string(logtofile, "", "");

namespace logging {
namespace logging_internal {
// https://stackoverflow.com/questions/77005
void handler(int sig, bool exit) {
  constexpr size_t depth = 64;
  constexpr size_t skip = 2;
  static void* array[depth + skip];

  // get void*'s for all entries on the stack
  size_t size = backtrace(array, depth + skip);

  // print out all the frames to stderr
  const char* name = "<UNKNOWN>";
  switch (sig) {
    case SIGSEGV:
      name = "(SIGSEGV)";
      break;
    case SIGTERM:
      name = "(SIGTERM)";
      break;
  }
  fprintf(stderr, "-------\nError: signal %d %s:\n", sig, name);
  backtrace_symbols_fd(array + skip, size - skip, /*STDERR_FILENO*/ 2);
  fprintf(stderr, "-------\n");
  if (exit) ::exit(1);
}
}  // namespace logging_internal

int InstallSignalhandler() {
  auto handler = +[](int s) { logging_internal::handler(s, true); };
  signal(SIGSEGV, handler);  // install our handler
  signal(SIGTERM, handler);  // install our handler
  return 0;
}

std::ostream& LogStream(LogLevel l) {
  if (FLAGS_logtostderr || l >= LogLevel::WARNING) return std::cerr;

  static auto& ret = *[]() -> std::ostream* {
    if (FLAGS_logtofile.empty()) FLAGS_logtofile = "/dev/null";

    auto out = new std::ofstream;
    out->open(FLAGS_logtofile, (std::ios::out | std::ios::trunc));
    CHECK(out->rdstate() == std::ios_base::goodbit)
        << "Failed to open logfile" << FLAGS_logtofile;
    return out;
  }();
  return ret;
}

void LogBase::DumpStack(std::ostream& o) {
  constexpr int depth = 30;
  static void* array[depth];
  int size = absl::GetStackTrace(array, depth, 1);
  auto names = backtrace_symbols(array, size);
  for (int i = 0; i < size; i++) {
    o << names[i] << '\n';
  }
  o << std::endl;
}

namespace logging_internal {
std::string Private::Demangle(const char* name) {
#ifdef __GNUG__
  int status = 1;
  auto res = abi::__cxa_demangle(name, nullptr, nullptr, &status);
  std::string ret =
      absl::StrReplaceAll((status == 0) ? res : name, {{" ", ""}});
  std::free(res);

  // A pile of string munging to make this look nice
  return absl::StrReplaceAll(ret,
                             {
                                 {"std::__cxx11::basic_string<char,std::char_"
                                  "traits<char>,std::allocator<char>>",
                                  "std::string"},
                             });
#else   //!__GNUG__
  // does nothing
  return name;
#endif  //__GNUG__
}
}  // namespace logging_internal
}  // namespace logging

namespace geo {
Eigen::Matrix3d Rotate(double ang, double azm) {
  const double ang_sin = sin(ang), ang_cos = cos(ang);
  const double azm_sin = sin(-azm), azm_cos = cos(-azm);
  return (Eigen::Matrix3d{} <<
          Eigen::RowVector3d{+ang_cos, +ang_sin * azm_cos, +ang_sin * azm_sin},
          Eigen::RowVector3d{-ang_sin, +ang_cos * azm_cos, +ang_cos * azm_sin},
          Eigen::RowVector3d{+0, -azm_sin, +azm_cos})
      .finished();
}

}  // namespace geo
