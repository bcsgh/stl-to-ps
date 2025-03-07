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

load("@com_github_bcsgh_parser_rules//parser:parser.bzl", "genlex", "genyacc")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

exports_files(["rule.bzl"])

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
        "@com_github_bcsgh_test_base//test_base:test_main",
        "@googletest//:gtest",
    ],
)

cc_library(
    name = "eigen_wrap",
    hdrs = ["eigen_wrap.h"],
    defines = ["EIGEN_INITIALIZE_MATRICES_BY_ZERO"],
    deps = [
        "@abseil-cpp//absl/log:check",
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
        "@abseil-cpp//absl/base",
        "@abseil-cpp//absl/strings",
    ],
)

cc_test(
    name = "center_test",
    timeout = "short",
    srcs = ["center_test.cc"],
    deps = [
        ":center",
        ":eigen_wrap",
        "@com_github_bcsgh_test_base//test_base:test_main",
        "@eigen",
        "@googletest//:gtest",
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
        "@abseil-cpp//absl/base",
        "@abseil-cpp//absl/base:core_headers",
        "@abseil-cpp//absl/log:log",
        "@abseil-cpp//absl/strings",
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
        "@abseil-cpp//absl/memory",
        "@com_github_bcsgh_test_base//test_base:test_main",
        "@googletest//:gtest",
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
        "@abseil-cpp//absl/flags:flag",
        "@com_github_bcsgh_parser_rules//parser:parser_support",
    ],
)

cc_library(
    name = "ps",
    srcs = ["ps.cc"],
    hdrs = ["ps.h"],
    deps = [
        ":eigen_wrap",
        ":geo",
        "@abseil-cpp//absl/flags:flag",
        "@abseil-cpp//absl/strings",
        "@abseil-cpp//absl/time",
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
        "@abseil-cpp//absl/strings",
        "@com_github_bcsgh_test_base//test_base:test_main",
        "@googletest//:gtest",
    ],
)

cc_library(
    name = "stl",
    srcs = ["stl.cc"],
    hdrs = ["stl.h"],
    deps = [
        ":eigen_wrap",
        ":geo",
        "@abseil-cpp//absl/log:log",
        "@re2",
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
        "@com_github_bcsgh_test_base//test_base:test_main",
        "@googletest//:gtest",
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
        "@abseil-cpp//absl/base",
        "@abseil-cpp//absl/base:core_headers",
        "@abseil-cpp//absl/flags:flag",
        "@abseil-cpp//absl/log:log",
        "@abseil-cpp//absl/strings",
        "@abseil-cpp//absl/strings:str_format",
        "@re2",
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
        "@abseil-cpp//absl/memory",
        "@abseil-cpp//absl/strings",
        "@com_github_bcsgh_test_base//test_base:test_main",
        "@googletest//:gtest",
    ],
)

cc_binary(
    name = "stl-to-ps",
    srcs = ["stl-to-ps-main.cc"],
    visibility = ["//visibility:public"],
    # linkopts = ["-rdynamic"],
    deps = [
        ":stl-to-ps_lib",
        "@abseil-cpp//absl/flags:flag",
        "@abseil-cpp//absl/flags:parse",
        "@abseil-cpp//absl/log:initialize",
        "@abseil-cpp//absl/log:log",
    ],
)

genyacc(
    name = "parser",
    src = "parser.y",
    cc = "parser.tab.cc",
    h = "parser.tab.h",
    loc = "location.hh",
)

genlex(
    name = "lexer",
    src = "lexer.l",
    cc = "lexer.yy.cc",
    h = "lexer.yy.h",
)
