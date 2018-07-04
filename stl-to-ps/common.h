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

#ifndef STL_TO_PS_COMMON_H_
#define STL_TO_PS_COMMON_H_

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <typeinfo>
#include <vector>

namespace logging {

int InstallSignalhandler();

enum class LogLevel {
  INFO = 0,
  WARNING = 1,
  ERROR = 2,
  QFATAL = 3,
  FATAL = 4,
};

std::ostream& LogStream(LogLevel);

class LogBase {
 protected:
  static void DumpStack(std::ostream& o);
};

template <LogLevel L>
struct AndNewline : private LogBase {
  ~AndNewline() {
    get() << std::endl;
    if (L >= LogLevel::FATAL) DumpStack(get());
    if (L >= LogLevel::QFATAL) exit(9);
  }
  std::ostream& get() { return LogStream(L); }
};

#define CHECK(x) \
  if ((x)) {     \
  } else         \
    LOG(FATAL) << "Fail: " << #x << ": "

#define LOG(_)                                          \
  ::logging::AndNewline<::logging::LogLevel::_>{}.get() \
      << std::string(#_, 1) << " " << __FILE__ << ":" << __LINE__ << ": "

template <class T>
std::string Demangle();

namespace logging_internal {
class Private {
  // Demanglers are hard to make tolerant to malicious inputs,
  // so restrict this to an API the just does names of built in types.
  static std::string Demangle(const char* name);

  template <class T>
  friend std::string logging::Demangle();
};
}  // namespace logging_internal

template <class T>
std::string Demangle() {
  return logging_internal::Private::Demangle(typeid(T).name());
}

}  // namespace logging

namespace base {
namespace internal {
inline const char* to_print(const std::string& c) { return c.c_str(); }

template <class U>
const U& to_print(const U& u) {
  return u;
}
}  // namespace internal

template <class... T>
std::string PrintF(std::string fmt, const T&... arg) {
  std::string ret(sizeof(void*) * 3 - 2, '\0');  // As big as will fit under SSO

  while (true) {
    auto b =
        snprintf(&ret[0], ret.size(), fmt.c_str(), internal::to_print(arg)...);
    CHECK(b >= 0);
    if (static_cast<size_t>(b) < ret.size()) {
      ret.resize(b);
      return ret;
    } else {
      ret.resize(b + 1);
    }
  }
}

template <class T>
T Take(T t) {
  return std::move(t);
}

template <class T>
T Take(T* t) {
  return std::move(*std::unique_ptr<T>(t));
}

}  // namespace base

#endif  // STL_TO_PS_COMMON_H_
