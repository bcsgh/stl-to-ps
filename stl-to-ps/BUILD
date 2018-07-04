# Copyright (c) 2017, Benjamin Shropshire,
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

cc_library(
  name = "test_main",
  srcs = ["test_main.cc"],
  deps = [
    ":common",
    "@com_github_gflags_gflags//:gflags",
    "@com_google_absl//absl/time",
    "@com_google_googletest//:gtest",
  ],
)

cc_library(
  name="geo",
  srcs=["geo.cc"],
  hdrs=["geo.h"],
  deps = [
    ":eigen_wrap",
  ],
)

cc_library(
  name="geo_test",
  srcs=["geo_test.cc"],
  deps = [
    ":geo",
    ":test_main",
    "@com_google_googletest//:gtest",
  ],
)

cc_library(
  name = "eigen_wrap",
  hdrs = ["eigen_wrap.h"],
  defines = ["EIGEN_INITIALIZE_MATRICES_BY_ZERO"],
  deps = [
    ":common",
    "@eigen//:eigen",
  ],
)

cc_library(
  name = "center",
  srcs = ["center.cc"],
  hdrs = ["center.h"],
  visibility = ["//visibility:public"],
  deps = [
    ":eigen_wrap",
    "@com_google_absl//absl/strings",
    "@com_google_absl//absl/base",
  ],
)

cc_test(
  name = "center_test",
  srcs = ["center_test.cc"],
  timeout="short",
  deps = [
    ":center",
    ":eigen_wrap",
    ":test_main",
    "@com_google_googletest//:gtest",
    "@eigen//:eigen",
  ],
)

cc_library(
  name = "ast",
  srcs = ["ast.cc"],
  hdrs = ["ast.h"],
  deps = [
    ":center",
    ":common",
    ":eigen_wrap",
    ":geo",
    "@com_google_absl//absl/base",
    "@com_google_absl//absl/base:core_headers",
  ],
)

cc_test(
  name = "ast_test",
  srcs = ["ast_test.cc"],
  timeout="short",
  deps = [
    ":ast",
    ":common",
    ":eigen_wrap",
    ":geo",
    ":test_main",
    "@com_google_absl//absl/memory",
    "@com_google_googletest//:gtest",
  ],
)

cc_library(
  name = "common",
  srcs = ["common.cc"],
  hdrs = ["common.h"],
  defines = ["EIGEN_INITIALIZE_MATRICES_BY_ZERO"],
  deps = [
    "@com_google_absl//absl/debugging:stacktrace",
    "@com_google_absl//absl/strings",
    "@com_github_gflags_gflags//:gflags",
  ],
)

cc_test(
  name = "common_test",
  srcs = ["common_test.cc"],
  timeout="short",
  deps = [
    ":common",
    ":test_main",
    "@com_google_googletest//:gtest",
  ],
)

cc_library(
  name = "parser_lib",
  srcs = [
    "parser.cc",
    ":lexer",
    ":parser",
  ],
  hdrs = [
    "parser.h",
    ":lexer",
    ":parser",
    "gen.lexer.h",
    "gen.parser.h",
  ],
  copts = [ # because bison
    "-fexceptions",
    "-Wno-sign-compare",
  ],
  deps = [
    ":ast",
    ":common",
    ":parser_support",
    "@com_github_gflags_gflags//:gflags",
  ],
)

cc_library(
  name = "parser_support",
  srcs = ["parser_support.cc"],
  hdrs = ["parser_support.h"],
)

cc_test(
  name = "parser_support_test",
  srcs = ["parser_support_test.cc"],
  timeout="short",
  deps = [
    ":common",
    ":parser_support",
    ":test_main",
    "@com_google_googletest//:gtest",
  ],
)

cc_library(
  name = "ps",
  srcs = ["ps.cc"],
  hdrs = ["ps.h"],
  deps = [
    ":common",
    ":eigen_wrap",
    ":geo",
    "@com_github_gflags_gflags//:gflags",
    "@com_google_absl//absl/strings",
    "@com_google_absl//absl/time",
  ],
)

cc_test(
  name = "ps_test", 
  srcs = ["ps_test.cc"],
  timeout="short",
  deps = [
    ":common",
    ":eigen_wrap",
    ":geo",
    ":ps",
    ":test_main",
    "@com_google_googletest//:gtest",
  ],
)

cc_library(
  name = "stl",
  srcs = ["stl.cc"],
  hdrs = ["stl.h"],
  deps = [
    ":common",
    ":eigen_wrap",
    ":geo",
    "@com_googlesource_code_re2//:re2",
  ],
)

cc_test(
  name = "stl_test", 
  srcs = ["stl_test.cc"],
  timeout="short",
  deps = [
    ":common",
    ":eigen_wrap",
    ":geo",
    ":stl",
    ":test_main",
    "@com_google_googletest//:gtest",
  ],
)

cc_library(
  name = "stl-to-ps_lib",
  srcs = [
      "stl-to-ps.cc",
      "stl-to-ps-impl.h",
  ],
  hdrs = ["stl-to-ps.h"],
  deps = [
    ":ast",
    ":center",
    ":eigen_wrap",
    ":geo",
    ":parser_lib",
    ":ps",
    ":stl",
    "@com_google_absl//absl/base",
    "@com_google_absl//absl/base:core_headers",
    "@com_google_absl//absl/strings",
    "@com_github_gflags_gflags//:gflags",
    "@com_googlesource_code_re2//:re2",
  ],
)

cc_test(
  name = "stl-to-ps_lib_test", 
  srcs = [
    "stl-to-ps_lib_test.cc",
    "stl-to-ps-impl.h"
  ],
  timeout="short",
  deps = [
    ":ast",
    ":common",
    ":eigen_wrap",
    ":geo",
    ":ps",
    ":stl",
    ":stl-to-ps_lib",
    ":test_main",
    "@com_google_absl//absl/memory",
    "@com_google_absl//absl/strings",
    "@com_google_googletest//:gtest",
  ],
)

cc_binary(
  name = "stl-to-ps",
  srcs = ["stl-to-ps-main.cc"],
  # linkopts = ["-rdynamic"],
  deps = [
    ":common",
    ":stl-to-ps_lib",
    "@com_github_gflags_gflags//:gflags",
  ],
  visibility = ["//visibility:public"],
)

load("//tools:parser.bzl", "genlex", "genyacc")

genyacc(
  name = "parser",
  src = "parser.y",
)

genlex(
  name = "lexer",
  src = "lexer.l",
)

