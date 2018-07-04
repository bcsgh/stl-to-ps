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

#include "stl-to-ps/ps.h"

#include <sstream>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "stl-to-ps/common.h"

namespace ps {
using testing::Eq;
using testing::HasSubstr;

TEST(PS, DocumentHeader) {
  // Just a smoke test.
  std::stringstream out(std::ios_base::out);
  DocumentHeader("doc", out);
}

TEST(PS, PageHeader) {
  // Just a smoke test.
  std::stringstream out(std::ios_base::out);
  PageHeader(1, 2, out);
}

TEST(PS, LinesToPs) {
  std::stringstream out(std::ios_base::out);
  // clang-format off
  LinesToPs({
    {Eigen::RowVector2d{1, 2}, {3, 4}},
    {Eigen::RowVector2d{5, 6}, {7, 8}},
  }, out);
  // clang-format on

  EXPECT_THAT(out.str(), Eq(R"(newpath 1 2 moveto 3 4 lineto stroke
newpath 5 6 moveto 7 8 lineto stroke
)"));
}

TEST(PS, ArcToPs) {
  std::stringstream out(std::ios_base::out);
  ArcToPs({{{1, 1}, 2, 0, 1.5}, {{2, 2}, 3, 3.1, 4.6}}, out);

  EXPECT_THAT(out.str(), Eq(R"(newpath 1 1 2 0 85.9437 arc stroke
newpath 2 2 3 177.617 263.561 arc stroke
)"));

  out.str({});  // reset
  ArcToPs({{geo::point::zero, 1, geo::point::x, geo::point::y},
           {geo::point::x + geo::point::y, 2, -geo::point::x, -geo::point::y}},
          out);

  EXPECT_THAT(out.str(), Eq(R"(newpath 0 0 1 0 90 arc stroke
newpath 1 1 2 -180 -90 arc stroke
)"));
}

TEST(PS, TextToPs) {
  std::stringstream out(std::ios_base::out);
  std::vector<Text> lines;
  TextToPs(lines, out);
  EXPECT_THAT(out.str(), Eq(""));

  Text t;
  t.at = {6, 9};
  t.str = "hello(\n)world";
  lines.emplace_back(t);

  TextToPs(lines, out);
  EXPECT_THAT(out.str(), Eq("newpath 6 9 moveto (hello\\(\n\\)world) show\n"));

  out.str({});  // reset
  t.center = true;
  lines[0] = ps::Text{std::move(t)};
  TextToPs(lines, out);
  EXPECT_THAT(out.str(),
              Eq("newpath 6 9 moveto (hello\\(\n\\)world) CenterText show\n"));

  out.str({});  // reset
  t.str = "hello \\351 world";
  t.raw = true;
  lines[0] = ps::Text{std::move(t)};
  TextToPs(lines, out);
  EXPECT_THAT(out.str(),
              Eq("newpath 6 9 moveto (hello \\351 world) CenterText show\n"));
}

TEST(PS, PageFooter) {
  // Just a smoke test.
  std::stringstream out(std::ios_base::out);
  PageFooter(out);
}

}  // namespace ps