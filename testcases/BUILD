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

ERROR_CASES = glob(["*.case"])
SUCCESS_CASES = glob(["*.example"])

[sh_test(
  name = "input_" + c.replace(".case", "_failure_test"),
  args = ["$(location //stl-to-ps:stl-to-ps)", "$(location %s)" % c],
  srcs = ["input_test.sh"],
  data = ["//stl-to-ps:stl-to-ps", c] + glob(["*.stl"]),
  timeout="short",
) for c in ERROR_CASES]

test_suite(
  name = "failure_tests",
  tests = ["input_" + c.replace(".case", "_failure_test") for c in ERROR_CASES],
)

load("//stl-to-ps:rule.bzl", "stl2pdf")
[stl2pdf(
    name = "input_" + c.replace(".", "_"),
    script = c,
    deps = glob(["*.stl"]),
) for c in SUCCESS_CASES]

load("//tools:build.bzl", "build_test")
[build_test(
  name = "input_" + c.replace(".example", "_success_test"),
  targets = [":input_" + c.replace(".", "_")],
) for c in SUCCESS_CASES]

test_suite(
  name = "success_tests",
  tests = [":input_" + c.replace(".example", "_success_test") for c in SUCCESS_CASES],
)
