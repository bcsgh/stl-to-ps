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

#include "stl-to-ps/stl.h"

#include <sstream>

#include "gtest/gtest.h"
#include "stl-to-ps/common.h"
#include "stl-to-ps/eigen_wrap.h"
#include "stl-to-ps/geo.h"

namespace stl2ps {

TEST(Edge, Smoke) {
  Edge e1, e2{{1, 2, 3}, {4, 5, 6}};
  std::stringstream o(std::ios_base::out);

  o << e1;
  EXPECT_EQ(o.str(), "{0 0 0,0 0 0}");

  o.str("");
  o << e2;
  EXPECT_EQ(o.str(), "{1 2 3,4 5 6}");
}

TEST(Facet, Smoke) {
  // Mostly to get full coverage of the generated functions
  Facet facet;
}

TEST(Facet, Basic) {
  std::vector<Eigen::RowVector3d> bits =  //
      {geo::point::x, geo::point::y, geo::point::z};

  Facet facet_p(Eigen::RowVector3d{1, 1, 1}, bits);
  Facet facet_n(-Eigen::RowVector3d{1, 1, 1}, bits);
}

TEST(STLFile, Smoke) {
  // Mostly to get full coverage of the generated functions
  STLFile file;
  auto file2 = file;
  (void)file2;
}

static constexpr char kGood[] = R"(solid test
  facet normal 1 1 1
    outer loop
      vertex 1.0 0.0 0
      vertex 0.0 1   0.0
      vertex 0   0.0 1.0
    endloop
  endfacet
  facet normal -1 0 -0.0
    outer loop
      vertex 0.0 0.0 0.0
      vertex 0.0 1.0 0.0
      vertex 0.0 0.0 1.0
    endloop
  endfacet
  facet normal 0.0 -2.0 0.0
    outer loop
      vertex 0.0 0.0 0.0
      vertex 1.0 0.0 0.0
      vertex 0.0 0.0 1.0
    endloop
  endfacet
  facet normal 0 0 -01
    outer loop
      vertex 0.0 0.0 0.0
      vertex 1.0 0.0 0.0
      vertex 0.0 1.0 0.0
    endloop
  endfacet
endsolid
)";

TEST(STLFile, Make) {
  // Empty case
  std::stringstream stream(std::ios_base::in);
  EXPECT_NE(STLFile::Make(stream), nullptr);

  // Basic case
  stream.str(kGood);
  auto f = STLFile::Make(stream);
  ASSERT_NE(f, nullptr);
  // TODO check things
}

TEST(STLFile, MakeErrors) {
  std::stringstream stream(std::ios_base::in);

  stream.str("unknown\n");
  ASSERT_EQ(STLFile::Make(stream), nullptr);

  stream.str(R"(solid test
  facet normal 0 0 0
    outer loop
      vertex 1.0 0.0 0
      vertex 0.0 1   0.0
      vertex 0   0.0 1.0
    endloop
  endfacet
endsolid
)");
  ASSERT_EQ(STLFile::Make(stream), nullptr);
}

TEST(STLFile, Rotate) {
  std::stringstream content(kGood, std::ios_base::in);
  auto f = STLFile::Make(content);
  auto r = f->Rotate(geo::matrix::XP);

  // TODO check stuff
}

TEST(STLFile, CullBackFace) {
  std::stringstream content(kGood, std::ios_base::in);
  auto f = STLFile::Make(content);

  f->CullBackFace();

  // TODO check stuff
}

TEST(STLFile, Limits) {
  std::stringstream content(kGood, std::ios_base::in);
  auto f = STLFile::Make(content);

  auto full = f->Limits();
  EXPECT_EQ(full, geo::Line(Eigen::RowVector2d{0, 0}, {1, 1}));

  // Test the empty case
  content.str("");
  auto empty = STLFile::Make(content)->Limits();
  EXPECT_EQ(empty, geo::Line(Eigen::RowVector2d{0, 0}, {0, 0}));
}

TEST(STLFile, ToLines) {
  std::stringstream content(kGood, std::ios_base::in);
  auto f = STLFile::Make(content);

  f->ToLines();

  // TODO check stuff
}

TEST(STLFile, Points) {
  std::stringstream content(kGood, std::ios_base::in);
  auto f = STLFile::Make(content);

  f->Points();

  // TODO check stuff
}

TEST(STLFile, Index) {
  // TODO
}

TEST(STLFile, EdgeCrosses) {
  // TODO
}

TEST(GenerateLines, TODO) {
  // TODO
}

}  // namespace stl2ps