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

#include "stl-to-ps/stl-to-ps-impl.h"
#include "stl-to-ps/stl-to-ps.h"

#include <sstream>

#include "absl/memory/memory.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "stl-to-ps/common.h"
#include "stl-to-ps/eigen_wrap.h"
#include "stl-to-ps/geo.h"

namespace stl2ps {

using testing::HasSubstr;

TEST(OutputPage, AddHeader) {
  OutputPage page;
  page.AddHeader("testing 1... 2... 3", 1, 2);

  // TODO check
}

TEST(ScriptToPS, RenderPages) {
  std::stringstream o(std::ios_base::out);

  RenderPages("", {{}}, o);
  EXPECT_THAT(o.str(), HasSubstr("/Courier findfont"));
  EXPECT_THAT(o.str(), HasSubstr("/CenterText"));
  EXPECT_THAT(o.str(), HasSubstr("<< /PageSize [792 612] >> setpagedevice"));
  EXPECT_THAT(o.str(), HasSubstr(".1 setlinewidth"));
  EXPECT_THAT(o.str(), HasSubstr("showpage"));
}

TEST(ScriptToPS, ScriptToPS) {
  std::stringstream o(std::ios_base::out);

  Document d;
  ScriptToPS("src", d, o);
  // TODO checks

  // TODO failure cases
}

TEST(ScriptToPS, LoadFiles) {
  std::map<std::string, std::unique_ptr<STLFile>> stl_files;
  stl_files[""] = nullptr;

  Document doc;  // Empty
  EXPECT_TRUE(LoadFiles(doc, &stl_files));
  EXPECT_EQ(stl_files.size(), 0);

  doc.models = {Model{"hello", "/dev/null", Loc{}}};  // One good
  EXPECT_TRUE(LoadFiles(doc, &stl_files));
  EXPECT_EQ(stl_files.size(), 1);

  doc.models.emplace_back("hello", "/dev/null", Loc{});  // duplicate name
  EXPECT_FALSE(LoadFiles(doc, &stl_files));

  // TODO better nameing
  doc.models = {Model{"hello", "bad_name_doesnt_exist", Loc{}}};  // Bad name
  EXPECT_FALSE(LoadFiles(doc, &stl_files));

  // TODO better nameing
  doc.models = {Model{"hello", "/dev/urandom", Loc{}}};  // Bad file
  EXPECT_FALSE(LoadFiles(doc, &stl_files));
}

TEST(ScriptToPS, GeneratePages) {
  Document doc;
  std::vector<OutputPage> pages;

  EXPECT_TRUE(GeneratePages({}, doc, &pages));
  EXPECT_EQ(pages.size(), 0);

  doc.pages.emplace_back(Loc{}, absl::make_unique<PageParts>());
  pages.clear();
  EXPECT_TRUE(GeneratePages({}, doc, &pages));
  EXPECT_EQ(pages.size(), 1);

  // TODO Move to AddHeader
  // Wrong type for name
  doc.pages[0].meta.emplace_back(Meta::New<int>("name", 1, Loc{}));
  EXPECT_FALSE(GeneratePages({}, doc, &pages));

  // TODO Move to AddHeader
  // Correct name
  doc.pages[0].meta[0] = Meta::New<std::string>("name", "x", Loc{});
  pages.clear();
  EXPECT_TRUE(GeneratePages({}, doc, &pages));
  EXPECT_EQ(pages.size(), 1);

  // Unknown prop.
  doc.pages[0].meta.emplace_back(Meta::New<std::string>("unknown", "", Loc{}));
  EXPECT_FALSE(GeneratePages({}, doc, &pages));

  // TODO page.draw
}

class DrawToPageTests : public ::testing::Test {
 public:
  static void SetRotation(DrawToPage* vis, Eigen::Matrix3d rotation) {
    vis->rotation_ = rotation;
  }

  static void SetPoints(DrawToPage* vis, const geo::point_set& p) {
    vis->points_ = p;
  }

  static bool GetCenter(DrawToPage* vis, const stl2ps::BaseDim& s,
                        std::map<std::string, Meta*>* seen,
                        Eigen::RowVector2d* a, Eigen::RowVector2d* d,
                        Eigen::RowVector2d* c, double* r) {
    return vis->GetCenter(s, seen, a, d, c, r);
  }

  static bool RenderDia(DrawToPage* vis, std::map<std::string, Meta*>* seen,
                        Eigen::RowVector2d a, Eigen::RowVector2d d,
                        Eigen::RowVector2d c, double r) {
    NodeI n{Loc{}};
    return vis->RenderDia(seen, n, a, d, c, r);
  }

  static bool RenderRad(DrawToPage* vis, std::map<std::string, Meta*>* seen,
                        Eigen::RowVector2d a, Eigen::RowVector2d d,
                        Eigen::RowVector2d c, double r) {
    NodeI n{Loc{}};
    return vis->RenderRad(seen, n, a, d, c, r);
  }
};

TEST(DrawToPage, AddArcs) {
  std::map<std::string, std::unique_ptr<STLFile>> target;
  DrawToPage vis(target);

  OutputPage page;
  vis.set_current_page(&page);

  vis.AddArcs({
      {Eigen::RowVector2d{1, 1}, 2, 0, 1.5},
      {Eigen::RowVector2d{2, 2}, 3, 3.1, 4.6},
  });
  // TODO check
}

TEST(DrawToPage, AddDims) {
  std::map<std::string, std::unique_ptr<STLFile>> target;
  DrawToPage vis(target);

  OutputPage page;
  vis.set_current_page(&page);

  std::vector<std::unique_ptr<BaseDim>> dims;
  EXPECT_TRUE(vis.AddDims(STLFile{}, dims, geo::matrix::I));

  dims.emplace_back(absl::make_unique<Dim>(Loc{}, MetaList{}));  // missing bits
  EXPECT_FALSE(vis.AddDims(STLFile{}, dims, geo::matrix::I));

  auto& d(*dims[0]);  // dim with wrong types
  d.meta_list.emplace_back(Meta::New<int>("at", 0, Loc{}));
  d.meta_list.emplace_back(Meta::New<int>("dir", 0, Loc{}));
  d.meta_list.emplace_back(Meta::New<int>("from", 0, Loc{}));
  d.meta_list.emplace_back(Meta::New<int>("to", 0, Loc{}));
  EXPECT_FALSE(vis.AddDims(STLFile{}, dims, geo::matrix::I));

  d.meta_list.clear();  // Correct dim
  d.meta_list.emplace_back(
      Meta::New<Point>("at", new Val(geo::point::zero, {}), Loc{}));
  d.meta_list.emplace_back(
      Meta::New<Point>("dir", new Val(geo::point::x, {}), Loc{}));
  d.meta_list.emplace_back(
      Meta::New<Point>("from", new Val(geo::point::zero, {}), Loc{}));
  d.meta_list.emplace_back(
      Meta::New<Point>("to", new Val(geo::point::zero, {}), Loc{}));
  EXPECT_TRUE(vis.AddDims(STLFile{}, dims, geo::matrix::I));

  // Format wrong type
  d.meta_list.emplace_back(Meta::New<int>("fmt", 1, Loc{}));
  EXPECT_FALSE(vis.AddDims(STLFile{}, dims, geo::matrix::I));

  // Invalid format
  *d.meta_list.rbegin() = Meta::New<std::string>("fmt", "%n", Loc{});
  EXPECT_FALSE(vis.AddDims(STLFile{}, dims, geo::matrix::I));

  // Valid Format
  *d.meta_list.rbegin() = Meta::New<std::string>("fmt", "%.0f", Loc{});
  EXPECT_TRUE(vis.AddDims(STLFile{}, dims, geo::matrix::I));

  // Extra args
  d.meta_list.emplace_back(Meta::New<int>("extra", 1, Loc{}));
  EXPECT_FALSE(vis.AddDims(STLFile{}, dims, geo::matrix::I));
  // TODO checks

  // TODO point::Invoke failure
}

TEST(DrawToPage, AddLines) {
  std::map<std::string, std::unique_ptr<STLFile>> target;
  DrawToPage vis(target);

  OutputPage page;
  vis.set_current_page(&page);

  vis.AddLines({
      {Eigen::RowVector2d{1, 2}, {3, 4}},  //
      {Eigen::RowVector2d{5, 6}, {7, 8}},
  });
  // TODO check
}

TEST(DrawToPage, AddText) {
  std::map<std::string, std::unique_ptr<STLFile>> target;
  DrawToPage vis(target);

  OutputPage page;
  vis.set_current_page(&page);

  vis.AddText({ps::Text(1, 2, "3"), ps::Text(4, 5, "678")});
  // TODO check
}

TEST_F(DrawToPageTests, VisitAngle) {
  std::map<std::string, std::unique_ptr<STLFile>> target;
  DrawToPage vis(target);
  SetRotation(&vis, geo::matrix::ZP);

  OutputPage page;
  vis.set_current_page(&page);

  Angle a{Loc{}, MetaList{}};

  EXPECT_FALSE(vis(a));  // Blank

  // Partial
  a.meta_list.emplace_back(
      Meta::New<Point>("at", new Val(geo::point::x, Loc{}), Loc{}));
  EXPECT_FALSE(vis(a));
  a.meta_list.emplace_back(
      Meta::New<Point>("apex", new Val(geo::point::zero, Loc{}), Loc{}));
  EXPECT_FALSE(vis(a));

  const int from = a.meta_list.size();
  a.meta_list.emplace_back(
      Meta::New<Point>("from_dir", new Val(geo::point::x, Loc{}), Loc{}));
  EXPECT_FALSE(vis(a));

  const int to = a.meta_list.size();
  a.meta_list.emplace_back(
      Meta::New<Point>("to_dir", new Val(geo::point::y, Loc{}), Loc{}));
  EXPECT_TRUE(vis(a));

  // Point backwards
  a.meta_list[from] =
      Meta::New<Point>("from_dir", new Val(geo::point::y, Loc{}), Loc{});
  a.meta_list[to] =
      Meta::New<Point>("to_dir", new Val(geo::point::x, Loc{}), Loc{});
  EXPECT_TRUE(vis(a));

  a.meta_list[from] =
      Meta::New<Point>("from_point", new Val(geo::point::y, Loc{}), Loc{});
  EXPECT_TRUE(vis(a));
  a.meta_list[to] =
      Meta::New<Point>("to_point", new Val(geo::point::x, Loc{}), Loc{});
  EXPECT_TRUE(vis(a));

  a.meta_list.emplace_back(Meta::New<int>("fmt", 0, Loc{}));
  EXPECT_FALSE(vis(a));

  *a.meta_list.rbegin() = Meta::New<std::string>("fmt", "%n", Loc{});
  EXPECT_FALSE(vis(a));

  *a.meta_list.rbegin() = Meta::New<std::string>("fmt", "%f", Loc{});
  EXPECT_TRUE(vis(a));

  // Wrong type
  std::unique_ptr<stl2ps::Meta> x =
      Meta::New<Eigen::RowVector2d>("to_dir", Eigen::RowVector2d{0, 0}, Loc{});
  std::swap(a.meta_list[to], x);
  EXPECT_FALSE(vis(a));
  std::swap(a.meta_list[to], x);

  // degenerate
  x = Meta::New<Point>("to_dir", new Val(geo::point::zero, Loc{}), Loc{});
  std::swap(a.meta_list[to], x);
  EXPECT_FALSE(vis(a));
  std::swap(a.meta_list[to], x);

  a.meta_list.emplace_back(
      Meta::New<Eigen::RowVector2d>("xxx", Eigen::RowVector2d{0, 0}, Loc{}));
  EXPECT_FALSE(vis(a));  // Unexpected

  *a.meta_list.rbegin() =
      Meta::New<Point>("from_point", new Val(geo::point::x, Loc{}), Loc{});
  EXPECT_FALSE(vis(a));

  *a.meta_list.rbegin() =
      Meta::New<Point>("to_point", new Val(geo::point::x, Loc{}), Loc{});
  EXPECT_FALSE(vis(a));
}

TEST_F(DrawToPageTests, VisitDim) {
  std::map<std::string, std::unique_ptr<STLFile>> target;
  DrawToPage vis(target);
  SetRotation(&vis, geo::matrix::ZP);

  OutputPage page;
  vis.set_current_page(&page);

  Dim d{Loc{}, MetaList{}};

  // Empty
  EXPECT_FALSE(vis(d));

  d.meta_list.emplace_back(
      Meta::New<Point>("at", new Val(geo::point::y, {}), Loc{}));
  d.meta_list.emplace_back(
      Meta::New<Point>("from", new Val(geo::point::x, {}), Loc{}));
  d.meta_list.emplace_back(
      Meta::New<Point>("to", new Val(geo::point::y, {}), Loc{}));
  d.meta_list.emplace_back(
      Meta::New<Point>("dir", new Val(geo::point::x, {}), Loc{}));
  EXPECT_TRUE(vis(d));
}

TEST(DrawToPage, VisitDraw) {
  std::map<std::string, std::unique_ptr<STLFile>> target;
  DrawToPage vis(target);

  OutputPage page;
  vis.set_current_page(&page);

  Draw d{Loc{}, absl::make_unique<std::string>(""),
         absl::make_unique<DrawList>()};

  // Wrong type
  d.meta_list.emplace_back(Meta::New<std::string>("scale", "", Loc{}));
  EXPECT_FALSE(vis(d));

  d.meta_list[0] =
      Meta::New<std::pair<int, int>>("scale", std::pair<int, int>{1, 2}, Loc{});

  // Unknown model;
  d.name = "foo";
  EXPECT_FALSE(vis(d));

  // Empty model
  target["foo"] = absl::make_unique<STLFile>();
  EXPECT_TRUE(vis(d));

  //////////////// View
  // Wrong type
  d.meta_list.emplace_back(Meta::New<int>("view", 1, Loc{}));
  EXPECT_FALSE(vis(d));

  *d.meta_list.rbegin() =
      Meta::New<Eigen::RowVector2d>("view", Eigen::RowVector2d{0, 0}, Loc{});
  EXPECT_TRUE(vis(d));

  // Bad name
  *d.meta_list.rbegin() = Meta::New<std::string>("view", "bad", Loc{});
  EXPECT_FALSE(vis(d));

  // Good name
  *d.meta_list.rbegin() = Meta::New<std::string>("view", "X+", Loc{});
  EXPECT_TRUE(vis(d));
  EXPECT_TRUE(vis(d));

  ////////////////
  // Change location
  d.meta_list.emplace_back(
      Meta::New<Eigen::RowVector2d>("@", Eigen::RowVector2d{1, 2}, Loc{}));
  EXPECT_TRUE(vis(d));

  ////////////////
  // Bad meta
  d.meta_list.emplace_back(Meta::New<std::string>("unknown", "", Loc{}));
  EXPECT_FALSE(vis(d));
}

TEST(DrawToPage, VisitText) {
  std::map<std::string, std::unique_ptr<STLFile>> target;
  DrawToPage vis(target);

  OutputPage page;
  vis.set_current_page(&page);

  Text t{Loc{}, absl::make_unique<std::string>(""),
         absl::make_unique<MetaList>()};

  // Empty
  EXPECT_TRUE(vis(t));

  // Change location
  t.meta_list.emplace_back(
      Meta::New<Eigen::RowVector2d>("@", Eigen::RowVector2d{1, 2}, Loc{}));
  EXPECT_TRUE(vis(t));

  // With text
  t.text = "hello\n\nworld";
  EXPECT_TRUE(vis(t));
  // TODO check page

  // Unknown prop.
  t.meta_list.emplace_back(Meta::New<std::string>("unknown", "", Loc{}));
  EXPECT_FALSE(vis(t));
}

TEST_F(DrawToPageTests, GetCenter) {
  std::map<std::string, Meta*> base, seen;
  std::map<std::string, std::unique_ptr<STLFile>> target;
  DrawToPage vis(target);
  SetRotation(&vis, geo::matrix::ZP);

  Rad obj{Loc{}};
  Eigen::RowVector2d at, dir, center;
  double r;
  // Missing bits
  EXPECT_FALSE(GetCenter(&vis, obj, &seen, &at, &dir, &center, &r));

  auto arg_at = Meta::New<Point>("at", new Val({1, 0, 0}, {}), Loc{});
  auto arg_dir = Meta::New<Point>("dir", new Val({1, 0.5, 0}, {}), Loc{});
  auto arg_center = Meta::New<Point>("center", new Val({0.1, 0, 0}, {}), Loc{});
  base = {{"at", arg_at.get()},
          {"dir", arg_dir.get()},
          {"center", arg_center.get()}};

  // Still not enought points.
  seen = base;
  EXPECT_FALSE(GetCenter(&vis, obj, &seen, &at, &dir, &center, &r));

  SetPoints(&vis, {{1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}});

  // Everything works
  seen = base;
  EXPECT_TRUE(GetCenter(&vis, obj, &seen, &at, &dir, &center, &r));

  EXPECT_EQ(at, Eigen::RowVector2d(1, 0));
  EXPECT_EQ(dir, Eigen::RowVector2d(1, 0.5));
  EXPECT_EQ(center, Eigen::RowVector2d(0, 0));
  EXPECT_EQ(r, 1);

  // A set of points that doesn't resolve to an arc.
  SetPoints(&vis, {{1, 0, 0}, {0, 0, 0}, {-1, 0, 0}});
  seen = base;
  EXPECT_FALSE(GetCenter(&vis, obj, &seen, &at, &dir, &center, &r));
}

TEST_F(DrawToPageTests, RenderDia) {
  std::map<std::string, std::unique_ptr<STLFile>> target;
  DrawToPage vis(target);

  OutputPage page;
  vis.set_current_page(&page);

  const Eigen::RowVector2d at{1, 0}, dir{1, 0.5}, center{0, 0};
  const double r = 1;
  std::unique_ptr<Meta> fmt;
  std::map<std::string, Meta*> seen;

  // Bad format
  seen["fmt"] = (fmt = Meta::New<int>("fmt", 1, Loc{})).get();
  EXPECT_FALSE(RenderDia(&vis, &seen, at, dir, center, r));

  // Bad format
  seen["fmt"] = (fmt = Meta::New<std::string>("fmt", "%n", Loc{})).get();
  EXPECT_FALSE(RenderDia(&vis, &seen, at, dir, center, r));

  // Works
  seen["fmt"] = (fmt = Meta::New<std::string>("fmt", "%.3f", Loc{})).get();
  EXPECT_TRUE(RenderDia(&vis, &seen, at, dir, center, r));

  seen["other"] = (fmt = Meta::New<std::string>("other", "NOPE", Loc{})).get();
  EXPECT_FALSE(RenderDia(&vis, &seen, at, dir, center, r));
}

TEST_F(DrawToPageTests, RenderRad) {
  std::map<std::string, std::unique_ptr<STLFile>> target;
  DrawToPage vis(target);

  OutputPage page;
  vis.set_current_page(&page);

  const Eigen::RowVector2d at{1, 0}, dir{1, 0.5}, center{0, 0};
  const double r = 1;
  std::unique_ptr<Meta> fmt;
  std::map<std::string, Meta*> seen;

  // Bad format
  seen["fmt"] = (fmt = Meta::New<int>("fmt", 1, Loc{})).get();
  EXPECT_FALSE(RenderRad(&vis, &seen, at, dir, center, r));

  // Bad format
  seen["fmt"] = (fmt = Meta::New<std::string>("fmt", "%n", Loc{})).get();
  EXPECT_FALSE(RenderRad(&vis, &seen, at, dir, center, r));

  // Works
  seen["fmt"] = (fmt = Meta::New<std::string>("fmt", "%.3f", Loc{})).get();
  EXPECT_TRUE(RenderRad(&vis, &seen, at, dir, center, r));

  seen["other"] = (fmt = Meta::New<std::string>("other", "NOPE", Loc{})).get();
  EXPECT_FALSE(RenderRad(&vis, &seen, at, dir, center, r));
}

}  // namespace stl2ps
