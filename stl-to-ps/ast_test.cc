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

#include "stl-to-ps/ast.h"

#include "absl/memory/memory.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "stl-to-ps/common.h"

namespace Eigen {
// Hack in a way to print points.
void PrintTo(const Eigen::RowVector3d& p, ::std::ostream* os) {
  *os << "[" << p << "]";
}
}  // namespace Eigen

namespace stl2ps {
namespace {
const int _i = logging::InstallSignalhandler();

using testing::Eq;
using testing::HasSubstr;

// Some types to check things out.
static const std::string fn = "foo";
struct loc_p {
  const std::string* filename = &fn;
  int line = 123;
};
struct loc {
  loc_p begin;
};

TEST(TestNodeI, Basic) {
  NodeI node(loc{});

  EXPECT_EQ(node.source_file(), "foo");
  EXPECT_EQ(node.source_line(), 123);
}

TEST(TestErrorMessage, Basic) {
  // TODO capture and check the result
  ErrorMessage(__FILE__, __LINE__, NodeI{loc{}}).get() << "Boo";
}

TEST(TestVal, Basic) {
  Val val({1, 2, 3}, loc{});

  Eigen::RowVector3d p, t1{1, 2, 3}, t2{2, 3, 1};
  EXPECT_TRUE(val.Invoke(geo::point_set{}, &p));
  EXPECT_EQ(p, t1);

  val.Rotate(geo::matrix::XP);
  EXPECT_TRUE(val.Invoke(geo::point_set{}, &p));
  EXPECT_EQ(p, t2);
}

TEST(TestImpl, Closest) {
  std::vector<Eigen::RowVector2d> ps{
    {-10,  10}, {0,  10}, {10,  10},
    {-10,   0}, {0,   0}, {10,   0},
    {-10, -10}, {0, -10}, {10, -10},
  };

  // Works when asked for notthing (or even less).
  EXPECT_THAT(point_impl::Closest(ps, -1, {1,0}), testing::IsEmpty());
  EXPECT_THAT(point_impl::Closest(ps, 0, {1,0}), testing::IsEmpty());

  EXPECT_THAT(point_impl::Closest(ps, 1, {1,0}),
              testing::ElementsAre(Eigen::RowVector2d{0,0}));

  EXPECT_THAT(point_impl::Closest(ps, 2, {1,0}),
              testing::UnorderedElementsAre(Eigen::RowVector2d{0,0},
                                            Eigen::RowVector2d{10,0}));

  EXPECT_THAT(point_impl::Closest(ps, 4, {1,0}),
              testing::UnorderedElementsAre(Eigen::RowVector2d{0,0},
                                            Eigen::RowVector2d{10,0},
                                            Eigen::RowVector2d{0,-10},
                                            Eigen::RowVector2d{0,10}));

  // Everything and more
  EXPECT_THAT(point_impl::Closest(ps, 10, {1,0}),
              testing::UnorderedElementsAreArray(ps));
}

TEST(TestImpl, Between) {
  std::vector<Eigen::RowVector2d> ps{
    {-10,  10}, {0,  10}, {10,  10},
    {-10,   0}, {0,   0}, {10,   0},
    {-10, -10}, {0, -10}, {10, -10},
  };

  EXPECT_THAT(point_impl::Between(ps, 0.0, 0.5, {1,0}), testing::IsEmpty());

  EXPECT_THAT(point_impl::Between(ps, 8.5, 9.5, {1,0}),
              testing::UnorderedElementsAre(Eigen::RowVector2d{10,0}));

  EXPECT_THAT(point_impl::Between(ps, 8.5, 10.5, {1,0}),
              testing::UnorderedElementsAre(Eigen::RowVector2d{10,0},
                                            Eigen::RowVector2d{0, -10},
                                            Eigen::RowVector2d{0, 10}));
}

TEST(TestPointFunc, Invoke) {
  PointFunc err(absl::make_unique<std::string>("error"), nullptr, loc{});
  EXPECT_FALSE(err.Invoke(geo::point_set{}, nullptr));  // Unknown func.
}

TEST(TestPointFunc, Near) {
  Val src({1, 2, 3}, loc{});
  PointFunc val(absl::make_unique<std::string>("near"),
                absl::make_unique<Val>(src), loc{});

  Eigen::RowVector3d p, t1{3, 3, 0};               // TODO why z = 0?
  EXPECT_FALSE(val.Invoke(geo::point_set{}, &p));  // Noting to find

  EXPECT_TRUE(val.Invoke(geo::point_set{t1}, &p));
  EXPECT_EQ(p, t1);

  val.Rotate(geo::matrix::XP);
  EXPECT_TRUE(val.Invoke(geo::point_set{t1}, &p));
  EXPECT_EQ(p, t1);

  // TODO test larger set
}

TEST(TestPointFunc, Center) {
  Val src({4, 4, 0}, loc{});
  PointFunc val(absl::make_unique<std::string>("center"),
                absl::make_unique<Val>(src), loc{});

  Eigen::RowVector3d p;
  EXPECT_FALSE(val.Invoke(geo::point_set{}, &p));  // Noting to find

  geo::point_set ps{
               {2, 4, 0},  // point is somwhere up here
    {0, 2, 0}, {2, 2, 0}, {4, 2, 0},
               {2, 0, 0},
  };
  EXPECT_TRUE(val.Invoke(ps, &p));
  EXPECT_EQ(p, Eigen::RowVector3d(3, 3, 0));
}

struct ABC {
  virtual void a() = 0;
};
struct DC : public ABC {
  void a() {}
};

TEST(TestMeta, New) {
  auto s = Meta::New<std::string>(new std::string{"name"}, std::string{"value"},
                                  loc{});
  auto c = Meta::New<ABC>(new std::string{"name"}, new DC, loc{});

  EXPECT_THAT(s->type_name(), Eq("std::string"));
  EXPECT_THAT(c->type_name(), HasSubstr("::ABC"));
  // TODO test more things.
}

TEST(TestMeta, As) {
  auto str = Meta::New<std::string>(new std::string{"name"},
                                    std::string{"value"}, loc{});

  EXPECT_FALSE(str->get<ABC>());
  EXPECT_TRUE(str->get<std::string>());
  std::string s;

  // EXPECT_FALSE(str->As<ABC>(nullptr));  // static_assert
  EXPECT_FALSE(str->As<int>(nullptr));
  EXPECT_TRUE(str->As<std::string>(&s));
  EXPECT_EQ(s, "value");

  ////
  auto abc = Meta::New<ABC>(new std::string{"name"}, new DC, loc{});

  EXPECT_FALSE(abc->get<std::string>());
  EXPECT_TRUE(abc->get<ABC>());
}

TEST(TestModel, TODO) {
  stl2ps::Model model(absl::make_unique<std::string>("hello"),
                      absl::make_unique<std::string>("hello"), Loc{});
}

TEST(TestDim, SmokeTest) {
  Dim smoke_test;  // Just to get coverage, nothing else to do here.
}

struct TestVisitor : public VisitDrawable {
  bool operator()(const Angle&) override { return true; }
  bool operator()(const Dia&) override { return true; }
  bool operator()(const Dim&) override { return true; }
  bool operator()(const Draw&) override { return true; }
  bool operator()(const Rad&) override { return true; }
  bool operator()(const Text&) override { return true; }
};

TEST(TestArc, TODO) {
  Angle a(BaseDim{});
  TestVisitor v;
  EXPECT_TRUE(a.Visit(&v));
}

TEST(TestDraw, TODO) {
  Draw d;
  d.Finish(absl::make_unique<std::string>("name"), Loc{});

  TestVisitor v;
  EXPECT_TRUE(d.Visit(&v));
}

TEST(TestText, TODO) {
  Text t;
  t.Finish(absl::make_unique<std::string>("name"), Loc{});

  TestVisitor v;
  EXPECT_TRUE(t.Visit(&v));
}

TEST(TestDia, TODO) {
  Dia d(BaseDim{});
  TestVisitor v;
  EXPECT_TRUE(d.Visit(&v));
}

TEST(TestDim, TODO) {
  Dim d;
  TestVisitor v;
  EXPECT_TRUE(d.Visit(&v));
}

TEST(TestRad, TODO) {
  Rad r(BaseDim{});
  TestVisitor v;
  EXPECT_TRUE(r.Visit(&v));
}

TEST(FreeFunctions, NewQuote) {
  auto q = absl::WrapUnique(NewQuote(R"("\\world\n")"));
  EXPECT_EQ(*q, "\\world\n");
}

TEST(FreeFunctions, GetMatrixByName) {
  Eigen::Matrix3d m = geo::matrix::I;

  EXPECT_FALSE(GetMatrixByName("W+", &m));
  EXPECT_EQ(m, geo::matrix::I);

  EXPECT_TRUE(GetMatrixByName("X+", &m));
  EXPECT_EQ(m, geo::matrix::XP);

  EXPECT_EQ(GetMatrixByAng({0, 0}), geo::matrix::I);
}

TEST(FreeFunctions, Document) {
  auto page = absl::make_unique<Page>();

  stl2ps::Document doc;
  doc.Add(absl::make_unique<stl2ps::Model>("hello", "hello", Loc{}));
  doc.Add(std::move(page));
}

}  // namespace
}  // namespace stl2ps
