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

load("@bazel_rules//parser:parser.bzl", "genlex", "genyacc")

cc_library(
    name = "test_main",
    srcs = ["test_main.cc"],
    deps = [
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "geo",
    srcs = ["geo.cc"],
    hdrs = ["geo.h"],
    deps = [
        ":eigen_wrap",
    ],
)

cc_library(
    name = "geo_test",
    srcs = ["geo_test.cc"],
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
        "@com_github_glog_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "center",
    srcs = ["center.cc"],
    hdrs = ["center.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":eigen_wrap",
        "@com_google_absl//absl/base",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "center_test",
    timeout = "short",
    srcs = ["center_test.cc"],
    deps = [
        ":center",
        ":eigen_wrap",
        ":test_main",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

cc_library(
    name = "ast",
    srcs = ["ast.cc"],
    hdrs = ["ast.h"],
    deps = [
        ":center",
        ":eigen_wrap",
        ":geo",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/base",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "ast_test",
    timeout = "short",
    srcs = ["ast_test.cc"],
    deps = [
        ":ast",
        ":eigen_wrap",
        ":geo",
        ":test_main",
        "@com_google_absl//absl/memory",
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
        "gen.lexer.h",
        "gen.parser.h",
        "parser.h",
        ":lexer",
        ":parser",
    ],
    copts = [
        # because bison
        "-fexceptions",
        "-Wno-sign-compare",
    ],
    deps = [
        ":ast",
        "@bazel_rules//parser:parser_support",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_library(
    name = "ps",
    srcs = ["ps.cc"],
    hdrs = ["ps.h"],
    deps = [
        ":eigen_wrap",
        ":geo",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
    ],
)

cc_test(
    name = "ps_test",
    timeout = "short",
    srcs = ["ps_test.cc"],
    deps = [
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
        ":eigen_wrap",
        ":geo",
        "@com_github_glog_glog//:glog",
        "@com_googlesource_code_re2//:re2",
    ],
)

cc_test(
    name = "stl_test",
    timeout = "short",
    srcs = ["stl_test.cc"],
    deps = [
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
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/base",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_googlesource_code_re2//:re2",
    ],
)

cc_test(
    name = "stl-to-ps_lib_test",
    timeout = "short",
    srcs = [
        "stl-to-ps-impl.h",
        "stl-to-ps_lib_test.cc",
    ],
    deps = [
        ":ast",
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
    visibility = ["//visibility:public"],
    # linkopts = ["-rdynamic"],
    deps = [
        ":stl-to-ps_lib",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
    ],
)

genyacc(
    name = "parser",
    src = "parser.y",
)

genlex(
    name = "lexer",
    src = "lexer.l",
)
