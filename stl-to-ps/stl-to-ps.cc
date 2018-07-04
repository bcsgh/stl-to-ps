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

#include "stl-to-ps/stl-to-ps.h"
#include "stl-to-ps/stl-to-ps-impl.h"

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "gflags/gflags.h"
#include "re2/re2.h"
#include "stl-to-ps/center.h"
#include "stl-to-ps/geo.h"
#include "stl-to-ps/eigen_wrap.h"
#include "stl-to-ps/parser.h"
#include "stl-to-ps/ps.h"
#include "stl-to-ps/stl.h"

DEFINE_string(load_path, "", "CSV list of root for loading files from");

namespace stl2ps {
namespace {
Meta* Take(const std::string& name, std::map<std::string, Meta*>* m) {
  auto i = m->find(name);
  if (i == m->end()) return nullptr;
  Meta* k = i->second;
  m->erase(i);
  return k;
}

bool Flatten(std::string name, const std::vector<std::unique_ptr<Meta>>& inp,
             std::map<std::string, Meta*>* seen) {
  bool ok = true;
  for (const auto& i : inp) {
    if (!seen->emplace(i->name, i.get()).second) {
      SYM_ERROR(*i) << "Duplicate " << name << " property '" << i->name << "'";
      ok = false;
    }
  }
  return ok;
}
}  // namespace

void OutputPage::AddHeader(const std::string& name, int page, int of) {
  float text_at = 784 - name.size() * ps::kFontSize * 0.6;

  ps::Text title;
  title.str = name;
  title.at.x() = text_at;
  title.at.y() = 8;
  text.push_back(title);

  constexpr double h = ps::kFontSize + 8;
  using p2 = Eigen::RowVector2d;
  lines.push_back(geo::Line{p2{(text_at - 3), 5}, {(text_at - 3), h}});
  lines.push_back(geo::Line{p2{(text_at - 3), h}, {787, h}});
}

void RenderPages(const std::string& src, const std::vector<OutputPage>& pages,
                 std::ostream& out) {
  ps::DocumentHeader(src, out);

  int page_number = 0;
  for (const auto& page : pages) {
    ps::PageHeader(++page_number, pages.size(), out);
    ps::TextToPs(page.text, out);
    ps::ArcToPs(page.arcs, out);
    ps::LinesToPs(page.lines, out);
    ps::PageFooter(out);
  }
}

bool ScriptToPS(const std::string& src, std::istream& in_orig,
                std::ostream& out) {
  std::string file_string(std::istreambuf_iterator<char>(in_orig), {});

  Document doc;
  if (int ret = Parse(src, file_string, &doc)) {
    LOG(ERROR) << "Parse failed with rc=" << ret;
    return false;
  }
  return ScriptToPS(src, doc, out);
}

bool LoadFiles(const Document& doc,
               std::map<std::string, std::unique_ptr<STLFile>>* stl_files) {
  stl_files->clear();
  bool ok = true;
  for (const auto& doc_model : doc.models) {
    std::string name = doc_model.name, file = doc_model.source;
    LOG(INFO) << "'" << name << "' '" << file << "'";
    auto added = stl_files->emplace(name, nullptr);
    if (!added.second) {
      SYM_ERROR(doc_model) << "Modele '" << name << "' was already loaded";
      ok = false;
      continue;
    }

    // Parse out the load path.
    static auto& load_paths = *new auto([] {
      std::vector<std::string> r = absl::StrSplit(FLAGS_load_path, ',');
      r.insert(r.begin(), ".");
      return r;
    }());

    std::ifstream model;
    if (file.find('/') == 0) {
      model.open(file, std::ios::in);
    } else {
      for (const auto& p : load_paths) {
        model.open(p + "/" + file, std::ios::in);
        if (model.rdstate() == std::ios_base::goodbit) break;
      }
    }
    if (model.rdstate() != std::ios_base::goodbit) {
      SYM_ERROR(doc_model) << "Failed to read model " << name  //
                           << " from '" << file << "'";
      ok = false;
      continue;
    }

    added.first->second = STLFile::Make(model);
    model.close();

    if (!added.first->second) {
      SYM_ERROR(doc_model) << "Failed to parse " << name << " '" << file << "'";
      ok = false;
      continue;
    }
  }
  return ok;
}

bool GeneratePages(
    const std::map<std::string, std::unique_ptr<STLFile>>& stl_files,
    const Document& doc, std::vector<OutputPage>* pages) {
  bool ok = true;
  DrawToPage visitor{stl_files};
  for (const auto& doc_page : doc.pages) {
    std::map<std::string, Meta*> seen;
    ok &= Flatten("page", doc_page.meta, &seen);

    pages->push_back({});
    auto& current_page = *pages->rbegin();

    std::string page_name = "";  // TODO move into AddHeader
    if (auto name = Take("name", &seen)) {
      if (!name->As(&page_name)) {
        SYM_ERROR(*name) << "Expected document name to be a string, got '"
                         << name->type_name() << "'";
        ok = false;
      }
    }

    visitor.set_current_page(&current_page);
    for (const auto& draw : doc_page.draws) {
      if (!draw->VisitNode(&visitor)) ok = false;
    }

    current_page.AddHeader(page_name, pages->size(), doc.pages.size());

    if (!seen.empty()) {
      SYM_ERROR(doc_page) << "Unexepcted properties for page " << pages->size();
      for (const auto& p : seen) {
        SYM_ERROR(*p.second) << p.second->name;
      }
      ok = false;
    }
  }
  return ok;
}

bool ScriptToPS(const std::string& src, const Document& doc,
                std::ostream& out) {
  std::map<std::string, std::unique_ptr<STLFile>> stl_files;
  if (!LoadFiles(doc, &stl_files)) return false;

  std::vector<OutputPage> pages;
  if (!GeneratePages(stl_files, doc, &pages)) return false;

  RenderPages(src, pages, out);
  return true;
}

bool DrawToPage::AddDims(const STLFile& file,
                         const std::vector<std::unique_ptr<BaseDim>>& dims,
                         Eigen::Matrix3d rotation) {
  points_ = file.Points();
  rotation_ = rotation;

  bool ok = true;
  for (const auto& dim : dims) ok &= dim->VisitNode(this);
  return ok;
}

bool DrawToPage::GetCenter(const BaseDim& dia, absl::string_view name,
                           Eigen::RowVector2d* ret_at,
                           Eigen::RowVector2d* ret_dir,
                           Eigen::RowVector2d* ret_center, double* ret_rad) {
  bool ok = true;

  std::map<std::string, Meta*> seen;
  ok &= Flatten("dia", dia.meta_list, &seen);
  auto p_at = Take("at", &seen);
  auto p_dir = Take("dir", &seen);
  auto p_center = Take("center", &seen);
  if (!seen.empty()) {
    SYM_ERROR(dia) << "Unexepcted properties for " << name;
    for (const auto& p : seen) SYM_ERROR(*p.second) << p.second->name;
    return false;
  }

  if (!p_at) {
    SYM_ERROR(dia) << "Missing 'at'";
    ok = false;
  }
  if (!p_dir) {
    SYM_ERROR(dia) << "Missing 'dir'";
    ok = false;
  }
  if (!p_center) {
    SYM_ERROR(dia) << "Missing 'center'";
    ok = false;
  }
  if (!ok) return false;
  if (points_.size() < 3) {
    SYM_ERROR(dia) << "Not enought points";
    return false;
  }

  Eigen::RowVector3d at, dir, target3;
  ok &= GetRotated(p_at, &at);
  ok &= GetRotated(p_center, &target3);
  if (p_dir) {
    ok &= GetRotated(p_dir, &dir);
  } else {
    dir = at - target3;
  }

  if (!ok) return false;

  Eigen::RowVector2d target = {target3.x(), target3.y()};

  // Flatten to 2d
  std::vector<Eigen::RowVector2d> points;
  for (const auto& p3 : points_) points.emplace_back(p3.x(), p3.y());

  // Grab the 3 closest posts to target
  auto near = point_impl::Closest(points, 3, target);

  // Find the center of the circle they form
  Eigen::RowVector2d center;
  double rad;
  if (!FindCircle(near, &center, &rad)) {
    SYM_ERROR(dia) << "Failed to find center";  // TODO
    return false;
  }

  // Find points of about the right distance from the presumed center.
  const double del = 0.1;
  near = point_impl::Between(points, rad * (1 - del), rad * (1 + del), center);

  // Find the center of the "circle" this (preumably) larger set of points form.
  if (!FindCircle(near, &center, &rad)) {
    SYM_ERROR(dia) << "Failed to find center";  // TODO
    return false;
  }

  *ret_at = {at.x(), at.y()};
  *ret_dir = {dir.x(), dir.y()};
  *ret_center = center;
  *ret_rad = rad;
  return true;
}

bool DrawToPage::GetRotated(stl2ps::Meta* p_in, Eigen::RowVector3d* p_out) {
  CHECK(rotation_ != Eigen::Matrix3d{}) << "Null rotation";

  if (auto p = p_in->get<Point>()) {
    p->Rotate(rotation_);
    if (!p->Invoke(points_, p_out)) {
      return false;
    }
  } else {
    SYM_ERROR(*p_in) << "Expected point for '" << p_in->name  //
                     << "' got " << p_in->type_name();
    return false;
  }
  return true;
}

//////// Visit drawable things.

bool DrawToPage::operator()(const Angle& dim) {
  bool ok = true;

  std::map<std::string, Meta*> seen;
  ok &= Flatten("angle", dim.meta_list, &seen);

  auto p_at = Take("at", &seen);
  auto p_apex = Take("apex", &seen);

  auto p_from_dir = Take("from_dir", &seen);
  auto p_from_point = Take("from_point", &seen);
  auto p_to_dir = Take("to_dir", &seen);
  auto p_to_point = Take("to_point", &seen);

  if (!seen.empty()) {
    SYM_ERROR(dim) << "Unexepcted properties for angle";
    for (const auto& p : seen) {
      SYM_ERROR(*p.second) << p.second->name;
    }
    return false;
  }

  if (!p_at) {
    SYM_ERROR(dim) << "Missing 'at'";
    ok = false;
  }
  if (!p_apex) {
    SYM_ERROR(dim) << "Missing 'apex'";
    ok = false;
  }
  if (!p_from_dir && !p_from_point) {
    SYM_ERROR(dim) << "Missing 'from_{dir,point}'";
    ok = false;
  }
  if (!p_to_dir && !p_to_point) {
    SYM_ERROR(dim) << "Missing 'to_{dir,point}'";
    ok = false;
  }
  if (!ok) return false;

  if (p_from_dir && p_from_point) {
    SYM_ERROR(*p_from_dir) << "Both 'from_{dir,point}' provided";
    ok = false;
  }
  if (p_to_dir && p_to_point) {
    SYM_ERROR(*p_to_dir) << "Both 'to_{dir,point}' provided";
    ok = false;
  }
  if (!ok) return false;

  Eigen::RowVector3d at, apex, from_dir, from_point, to_dir, to_point;
  ok &= GetRotated(p_at, &at);
  ok &= GetRotated(p_apex, &apex);

  if (p_from_dir) {
    ok &= GetRotated(p_from_dir, &from_dir);
    from_dir = from_dir.normalized();
    from_point = apex;
  } else {
    ok &= GetRotated(p_from_point, &from_point);
    from_dir = (from_point - apex).normalized();
  }
  if (p_to_dir) {
    ok &= GetRotated(p_to_dir, &to_dir);
    to_dir = to_dir.normalized();
    to_point = apex;
  } else {
    ok &= GetRotated(p_to_point, &to_point);
    to_dir = (to_point - apex).normalized();
  }

  if (!ok) return false;

  auto c = to_dir.cross(from_dir);
  if (c.z() == 0) {
    SYM_ERROR(dim) << "Degenerate angle provided.";
    return false;
  }
  if (c.z() > 0) {
    using std::swap;
    swap(to_dir, from_dir);
    swap(to_point, from_point);
  }

  double l = (at - apex).norm();
  double gap = 2 / proj.scale;

  Eigen::RowVector3d fa = from_dir.normalized() / proj.scale;
  Eigen::RowVector3d ta = to_dir.normalized() / proj.scale;
  auto dot = from_dir.dot(to_dir);
  Eigen::RowVector3d fb =
      (to_dir - from_dir * dot / from_dir.norm()).normalized() / proj.scale;
  Eigen::RowVector3d tb =
      (from_dir - to_dir * dot / to_dir.norm()).normalized() / proj.scale;
  Eigen::RowVector3d fpt = apex + from_dir * l;
  Eigen::RowVector3d tpt = apex + to_dir * l;

  using p3 = Eigen::RowVector3d;
  AddLines({
      {p3{from_point + from_dir * gap}, apex + from_dir * (l + gap * 2)},
      {p3{to_point + to_dir * gap}, apex + to_dir * (l + gap * 2)},

      {fpt, fpt + fa * +2 + fb * 6},
      {fpt, fpt + fa * -2 + fb * 6},
      {tpt, tpt + ta * +2 + tb * 6},
      {tpt, tpt + ta * -2 + tb * 6},
  });
  AddArcs({
      {apex, l, from_dir, to_dir},
  });

  ps::Text t;
  double ang = std::acos(from_dir.dot(to_dir.normalized())) * 180 / geo::PI;
  t.at.x() = at.x();
  t.at.y() = at.y();
  t.str = base::PrintF("%.1f deg", ang);
  t.center = true;
  AddText({t});

  return true;
}

bool DrawToPage::operator()(const Dia& dia) {
  Eigen::RowVector2d center, at, dir;
  double r;

  return GetCenter(dia, "dia", &at, &dir, &center, &r) &&
         RenderDia(center, at, dir, r);
}

bool DrawToPage::RenderDia(Eigen::RowVector2d center, Eigen::RowVector2d at,
                           Eigen::RowVector2d dir, double r) {
  dir = dir.normalized();

  Eigen::RowVector2d a = center + dir * r;
  Eigen::RowVector2d b = center - dir * r;

  Eigen::RowVector2d direct = at - center;
  double len = std::min(direct.x() / dir.x(), direct.y() / dir.y());
  if (len > 1) {
    Eigen::RowVector2d c = center + dir * len;
    AddLines({{b, c}, {c, at}});
  } else {
    AddLines({{b, a}, {a, at}});
  }

  dir /= proj.scale;
  Eigen::RowVector2d cross{-dir.y(), dir.x()};
  AddLines({
      {a, a + (-6 * dir + +2 * cross)},  // Arrow heads
      {a, a + (-6 * dir + -2 * cross)},
      {b, b + (+6 * dir + +2 * cross)},
      {b, b + (+6 * dir + -2 * cross)},
  });

  ps::Text t;
  t.at = at;
  t.str = base::PrintF("\\351%.3f", r * 2);
  t.raw = true;
  t.center = true;
  // Offset for width of text.  TODO deal with left side placements
  t.at.x() += (ps::kFontSize * 0.3 * (t.str.length() - 2)) / proj.scale;

  AddText({t});

  return true;
}

bool DrawToPage::operator()(const Dim& dim) {
  bool ok = true;

  std::map<std::string, Meta*> seen;
  ok &= Flatten("dim", dim.meta_list, &seen);
  auto p_at = Take("at", &seen);
  auto p_dir = Take("dir", &seen);
  auto p_from = Take("from", &seen);
  auto p_to = Take("to", &seen);
  if (!seen.empty()) {
    SYM_ERROR(dim) << "Unexepcted properties for dim";
    for (const auto& p : seen) {
      SYM_ERROR(*p.second) << p.second->name;
    }
    return false;
  }

  if (!p_at) {
    SYM_ERROR(dim) << "Missing 'at'";
    ok = false;
  }
  if (!p_dir) {
    SYM_ERROR(dim) << "Missing 'dir'";
    ok = false;
  }
  if (!p_from) {
    SYM_ERROR(dim) << "Missing 'from'";
    ok = false;
  }
  if (!p_to) {
    SYM_ERROR(dim) << "Missing 'to'";
    ok = false;
  }
  if (!ok) return false;

  Eigen::RowVector3d at, dir, from, to;
  ok &= GetRotated(p_at, &at);
  ok &= GetRotated(p_dir, &dir);
  ok &= GetRotated(p_from, &from);
  ok &= GetRotated(p_to, &to);

  if (dir == geo::point::zero) {
    SYM_ERROR(*p_dir) << "Dimention's 'dir' must not be the zero vector";
    return false;
  }
  dir = dir.normalized();

  if (!ok) return false;

  // TODO this should all be in plane.
  Eigen::RowVector3d from_at = from - at;
  Eigen::RowVector3d to_at = to - at;
  Eigen::RowVector3d arrow_from = at + dir * dir.dot(from_at);
  Eigen::RowVector3d arrow_to = at + dir * dir.dot(to_at);

  Eigen::RowVector3d length = arrow_to - arrow_from;
  double dim_value = length.norm();
  ps::Text t;
  t.at.x() = at.x();
  t.at.y() = at.y();
  t.str = base::PrintF("%.3f", dim_value);
  t.center = true;
  AddText({t});

  // Local axis for features
  Eigen::RowVector3d a =  // 1 pt along the dim
      Eigen::RowVector3d{length.x(), length.y(), 0}.normalized() / proj.scale;
  Eigen::RowVector3d b = {-a.y(), a.x(), 0};  // 1 pt across the dim

  // Which direction do the extention lines go?
  double from_sign = from_at.dot(b) > 0 ? +1 : -1;
  double to_sign = to_at.dot(b) > 0 ? +1 : -1;

  // Find the offsets to build a bounding box around the text.
  double width = ps::kFontSize * 0.3 * t.str.length() + 1;
  double height = ps::kFontSize * 0.333 + 1;
  auto gap = std::min(std::abs(width / a.x()), std::abs(height / a.y()));
  gap /= proj.scale;

  // Dimention lines
  if (gap * 2 < dim_value * proj.scale) {  // cmp the gap and dim (in pt's)
    AddLines({
        {arrow_from, at - a * gap}, {at + a * gap, arrow_to},
    });
  } else {
    AddLines({
        {arrow_from, arrow_from - a * 10}, {arrow_to, arrow_to + a * 10},
    });
    a = -a;
  }

  using p3 = Eigen::RowVector3d;
  AddLines({
      // Extention lines.
      {p3{arrow_from - b * 4 * from_sign}, from - b * 2 * from_sign},
      {p3{arrow_to - b * 4 * to_sign}, to - b * 2 * to_sign},

      // From arrow
      {arrow_from, arrow_from + (a * +6 + b * +2)},
      {arrow_from, arrow_from + (a * +6 + b * -2)},
      // To arrow
      {arrow_to, arrow_to + (a * -6 + b * +2)},
      {arrow_to, arrow_to + (a * -6 + b * -2)},
  });

  return true;
}

bool DrawToPage::operator()(const Draw& draw) {
  std::map<std::string, Meta*> seen;
  bool ok = Flatten("draw", draw.meta_list, &seen);
  if (!ok) return false;

  std::pair<int, int> scales = {1, 1};
  if (auto s = Take("scale", &seen)) {
    if (!s->As(&scales)) {
      SYM_ERROR(*s) << "Wrong type for scale";
      return false;
    }
  }
  proj.scale = 72.0 * scales.first / scales.second;

  const auto file = stl_files.find(draw.name);
  if (file == stl_files.end()) {
    SYM_ERROR(draw) << "Unknown model '" << draw.name << "'";
    return false;
  }

  Eigen::Matrix3d rotation = geo::matrix::ZP;
  if (auto view = Take("view", &seen)) {
    Eigen::RowVector2d loc;
    std::string name;
    if (view->As(&loc)) {
      rotation = stl2ps::GetMatrixByAng(loc);
    } else if (view->As(&name)) {
      if (!stl2ps::GetMatrixByName(name, &rotation)) {
        SYM_ERROR(*view) << "Unknown view name: '" << name << "'";
        return false;
      }
    } else {
      SYM_ERROR(*view) << "Unexpect view type: " << view->type_name();
      return false;
    }
  }

  Eigen::RowVector2d loc = {0, 0};
  if (auto at = Take("@", &seen)) {
    CHECK(at->As(&loc)) << at->type_name();
    loc *= 72.0;
    loc += Eigen::RowVector2d{792 / 2, 612 / 2};
  }

  // project the model.
  auto projection = file->second->Rotate(rotation);
  projection.CullBackFace();

  // Setup the context.
  auto page_limit = projection.Limits();
  proj.d = loc - (page_limit.o + page_limit.t) / 2 * proj.scale;

  // Render things out.
  AddLines(projection.ToLines());

  if (!AddDims(projection, draw.dims, rotation)) return false;

  if (!seen.empty()) {
    SYM_ERROR(draw) << "Unexepcted properties for draw";
    for (const auto& p : seen) {
      SYM_ERROR(*p.second) << p.second->name;
    }
    return false;
  }
  return true;
}

bool DrawToPage::operator()(const Rad& rad_) {
  Eigen::RowVector2d center, dir, at;
  double r;

  return GetCenter(rad_, "rad", &at, &dir, &center, &r) &&
         RenderRad(center, at, dir, r);
}

bool DrawToPage::RenderRad(Eigen::RowVector2d center, Eigen::RowVector2d at,
                           Eigen::RowVector2d dir, double r) {
  dir = dir.normalized();

  Eigen::RowVector2d a = center + dir * r;

  // TODO, Do a better job here.
  Eigen::RowVector2d direct = at - center;
  double len = std::min(direct.x() / dir.x(), direct.y() / dir.y());
  if (len > 1) {
    Eigen::RowVector2d b = center + dir * len;
    AddLines({{center, b}, {b, at}});
  } else {
    AddLines({{center, a}, {a, at}});
  }

  dir /= proj.scale;
  Eigen::RowVector2d cross{-dir.y(), dir.x()};
  AddLines({
      {a, a + (-6 * dir + +2 * cross)},  // Arrow head
      {a, a + (-6 * dir + -2 * cross)},
  });

  ps::Text t;
  t.at = at;
  t.str = base::PrintF("R%.3f", r);
  t.center = true;
  // Offset for width of text.  TODO deal with left side placements
  t.at.x() += (ps::kFontSize * 0.3 * (t.str.length() + 2)) / proj.scale;

  AddText({t});

  return true;
}

bool DrawToPage::operator()(const Text& text) {
  std::map<std::string, Meta*> seen;
  bool ok = Flatten("draw", text.meta_list, &seen);
  if (!ok) return false;

  ps::Text t;
  t.at = {0, 0};

  if (auto at = Take("@", &seen)) {
    Eigen::RowVector2d loc = {0, 0};
    CHECK(at->As(&loc)) << at->type_name();
    t.at += loc * 72.0;
  }

  t.at += Eigen::RowVector2d{792 / 2, 612 / 2};
  for (const auto& s : absl::StrSplit(text.text, "\n")) {
    if (!s.empty()) {
      t.str = std::string(s);
      current_page->text.push_back(t);
    }
    t.at.y() -= 12;
  }
  if (!seen.empty()) {
    SYM_ERROR(text) << "Unexepcted properties for text";
    for (const auto& p : seen) {
      SYM_ERROR(*p.second) << p.second->name;
    }
    return false;
  }
  return true;
}

///////// Add things to the page.

void DrawToPage::AddLines(const std::vector<geo::Line>& lines) {
  CHECK(current_page != nullptr);
  current_page->lines.reserve(
      std::max(current_page->lines.capacity(),
               current_page->lines.size() + lines.size()));
  for (auto l : lines) {
    l.o *= proj.scale;
    l.t *= proj.scale;
    l.o += proj.d;
    l.t += proj.d;

    current_page->lines.push_back(l);
  }
}

void DrawToPage::AddArcs(const std::vector<geo::Arc>& arcs) {
  CHECK(current_page != nullptr);
  current_page->arcs.reserve(std::max(current_page->arcs.capacity(),
                                      current_page->arcs.size() + arcs.size()));
  for (auto a : arcs) {
    a.center *= proj.scale;
    a.center += proj.d;
    a.r *= proj.scale;

    current_page->arcs.push_back(a);
  }
}

void DrawToPage::AddText(const std::vector<ps::Text>& text) {
  CHECK(current_page != nullptr);
  current_page->text.reserve(std::max(current_page->text.capacity(),
                                      current_page->text.size() + text.size()));
  for (auto t : text) {
    t.at *= proj.scale;
    t.at += proj.d;
    current_page->text.push_back(t);
  }
}

}  // namespace stl2ps
