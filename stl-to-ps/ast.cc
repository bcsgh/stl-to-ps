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

#include <cxxabi.h>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_replace.h"
#include "stl-to-ps/center.h"
#include "stl-to-ps/eigen_wrap.h"
#include "stl-to-ps/geo.h"

namespace stl2ps {

bool PointFunc::Invoke(const geo::point_set& ps, Eigen::RowVector3d* ret) {
  if (name_ == "near") {
    return Near(ps, ret);
  }
  if (name_ == "center") {
    return Center(ps, ret);
  }
  SYM_ERROR(*this) << "Unknown function '" << name_ << "'";
  return false;
}

bool PointFunc::Near(const geo::point_set& ps, Eigen::RowVector3d* ret) {
  if (ps.empty()) return false;

  Eigen::RowVector3d target;
  if (!p_->Invoke(ps, &target)) return false;
  if (ps.find(target) != ps.end()) {
    *ret = target;
    return true;
  }
  target = {target.x(), target.y(), 0};

  double r = std::numeric_limits<double>::max();

  for (auto x : ps) {
    x = {x.x(), x.y(), 0};
    double d = (target - x).squaredNorm();
    if (r > d) {
      r = d;
      *ret = x;
    }
  }
  return true;
}

namespace point_impl {

// Find the `count` points in `ps` closest to `to`.
std::vector<Eigen::RowVector2d> Closest(
    const std::vector<Eigen::RowVector2d>& ps, int count,
    Eigen::RowVector2d to) {
  if (count < 1) return {};

  std::multimap<double, Eigen::RowVector2d> c;
  for (const auto& p : ps) {
    c.emplace((p - to).squaredNorm(), p);
    while (static_cast<int>(c.size()) > count) c.erase(--c.end());
  }
  std::vector<Eigen::RowVector2d> ret;
  for (const auto& p : c) ret.push_back(p.second);
  return ret;
}

// Find all points in `ps` that are betwwwn `min` and `max` from `to`.
std::vector<Eigen::RowVector2d> Between(
    const std::vector<Eigen::RowVector2d>& ps, double min, double max,
    Eigen::RowVector2d to) {
  if (max < 0 || min > max) return {};

  std::vector<Eigen::RowVector2d> ret;
  for (const auto& p : ps) {
    double r2 = (p - to).squaredNorm();
    if (min * min < r2 && r2 < max * max) ret.emplace_back(p);
  }
  return ret;
}

}  // namespace point_impl

// NOTE: this uses a rather primitive huristic
bool PointFunc::Center(const geo::point_set& ps, Eigen::RowVector3d* ret) {
  if (ps.size() < 3) return false;

  Eigen::RowVector3d target3;
  if (!p_->Invoke(ps, &target3)) return false;
  Eigen::RowVector2d target = {target3.x(), target3.y()};

  // Flatten to 2d
  std::vector<Eigen::RowVector2d> points;
  for (const auto& p3 : ps) points.emplace_back(p3.x(), p3.y());

  // Grab the 3 closest posts to target
  auto near = point_impl::Closest(points, 3, target);

  // Find the center of the circle they form
  Eigen::RowVector2d center;
  double rad;
  if (!FindCircle(near, &center, &rad)) return false;

  // Find points of about the right distance from the presumed center.
  const double del = 0.1;
  near = point_impl::Between(points, rad * (1 - del), rad * (1 + del), center);

  // Find the center of the "circle" this (preumably) larger set of points form.
  if (!FindCircle(near, &center, &rad)) return false;

  // Convert back to 3d and return.
  *ret = {center.x(), center.y(), 0};
  return true;
}

std::string* NewQuote(std::string ret) {
  size_t t = 0;
  for (size_t f = 1; f < ret.length() - 1; t++, f++) {
    if (ret[f] != '\\') {
      ret[t] = ret[f];
    } else {
      switch (ret[++f]) {
        case 'n':
          ret[t] = '\n';
          break;
        default:
          ret[t] = ret[f];
          break;
      }
    }
  }
  ret.resize(t);

  return new std::string(std::move(ret));
}

void Document::Add(std::unique_ptr<Model> m) {
  models.emplace_back(std::move(*m));
}

void Document::Add(std::unique_ptr<Page> p) {
  pages.emplace_back(std::move(*p));
}

bool GetMatrixByName(const std::string& s, Eigen::Matrix3d* m) {
  static const std::map<std::string, Eigen::Matrix3d> kViews = {
      {"X+", geo::matrix::XP}, {"X-", geo::matrix::XN},  //
      {"Y+", geo::matrix::YP}, {"Y-", geo::matrix::YN},  //
      {"Z+", geo::matrix::ZP}, {"Z-", geo::matrix::ZN},
  };

  auto found = kViews.find(s);
  if (found == kViews.end()) return false;
  *m = found->second;
  return true;
}

Eigen::Matrix3d GetMatrixByAng(const Eigen::RowVector2d& a) {
  return geo::Rotate(a.x() * geo::PI / 180, a.y() * geo::PI / 180);
}

bool Angle::Visit(VisitDrawable* v) { return (*v)(*this); }
bool Dia::Visit(VisitDrawable* v) { return (*v)(*this); }
bool Dim::Visit(VisitDrawable* v) { return (*v)(*this); }
bool Draw::Visit(VisitDrawable* v) { return (*v)(*this); }
bool Rad::Visit(VisitDrawable* v) { return (*v)(*this); }
bool Text::Visit(VisitDrawable* v) { return (*v)(*this); }

std::string Meta::UnsafeDemangle(const char* name) {
#ifdef __GNUG__
  int status = 1;
  auto res = abi::__cxa_demangle(name, nullptr, nullptr, &status);
  std::string ret =
      absl::StrReplaceAll((status == 0) ? res : name, {{" ", ""}});
  std::free(res);

  // A pile of string munging to make this look nice
  return absl::StrReplaceAll(ret,
                             {
                                 {"std::__cxx11::basic_string<char,std::char_"
                                  "traits<char>,std::allocator<char>>",
                                  "std::string"},
                             });
#else   //!__GNUG__
  // does nothing
  return name;
#endif  //__GNUG__
}

}  // namespace stl2ps
