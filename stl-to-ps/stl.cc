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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <map>
#include <memory>
#include <set>

#include "glog/logging.h"
#include "re2/re2.h"
#include "stl-to-ps/eigen_wrap.h"
#include "stl-to-ps/geo.h"

namespace stl2ps {
constexpr double kMinLineLen = 0;
constexpr bool show_hidden = false;
constexpr bool show_invisible = false;
constexpr double min_ang = .98;
using std::swap;

std::ostream& operator<<(std::ostream& o, const Edge& e) {
  return o << '{' << e.first << ',' << e.second << '}';
}

////////////////////////////////

Facet::Facet(const Eigen::RowVector3d& n,
             const std::vector<Eigen::RowVector3d>& e)
    : norm((e[1] - e[0]).cross(e[2] - e[0]).normalized()),
      edge{e[0], e[1], e[2]} {
  if (n.dot(norm) < 0) {
    swap(edge[0], edge[2]);
    norm = norm * -1;
  }

  // Take the average to reduce rounding error
  r = (norm.dot(edge[0]) + norm.dot(edge[1]) + norm.dot(edge[2])) / 3;
}

////////////////////////////////

std::unique_ptr<STLFile> STLFile::Make(std::istream& in) {
  std::unique_ptr<STLFile> ret{new STLFile};

  std::string line;
  int line_number = 0, solid = 0, loop = 0;

  Eigen::RowVector3d norm = geo::point::zero;
  std::vector<Eigen::RowVector3d> points;
  double x, y, z;
  while (std::getline(in, line)) {
    line_number++;

    // Ignore
    static const auto& re_solid = *new RE2(R"( *(solid|endsolid)( .*)?)");
    if (RE2::FullMatch(line, re_solid)) {
      solid++;
      continue;
    }

    static const auto& re_oloop = *new RE2(R"( *(outer loop|endloop) *)");
    if (RE2::FullMatch(line, re_oloop)) {
      loop++;
      continue;
    }

    // Start of facet
    static const auto& re_facet = *new RE2(
        R"( *facet normal +([-.e0-9]+) +([-.e0-9]+) +([-.e0-9]+) *)");
    if (RE2::FullMatch(line, re_facet, &x, &y, &z)) {
      norm = {x, y, z};
      points.clear();
      continue;
    }

    static const auto& re_efacet = *new RE2(R"( *endfacet *)");
    if (RE2::FullMatch(line, re_efacet)) {
      if (norm == geo::point::zero) {
        LOG(WARNING) << line_number << ": missing normal";
        continue;
      } else {
        ret->facets.emplace_back(norm, points);
      }
      norm = geo::point::zero;
      points.clear();
      continue;
    }

    static const auto& re_vertex = *new RE2(
        R"( *vertex +([-.e0-9]+) +([-.e0-9]+) +([-.e0-9]+) *)");
    if (RE2::FullMatch(line, re_vertex, &x, &y, &z)) {
      points.push_back({x, y, z});
      continue;
    }

    LOG(ERROR) << line_number << ": Unknown line: '" << line << "'";
    return nullptr;
  }
  LOG(INFO) << "lines:" << line_number << " solid:" << solid << " loop:" << loop
            << " facets:" << ret->facets.size();
  ret->Index();
  return ret;
}

STLFile STLFile::Rotate(const Eigen::Matrix3d& m) const {
  STLFile ret = *this;
  for (auto& f : ret.facets) {
    f.norm = f.norm * m;
    f.edge[0] = f.edge[0] * m;
    f.edge[1] = f.edge[1] * m;
    f.edge[2] = f.edge[2] * m;
  }
  ret.CullBackFace();
  ret.Index();
  return ret;
}

void STLFile::CullBackFace() {
  int i = 0, j = facets.size();
  for (; i < j; i++) {
    if (facets[i].norm.z() <= 0) swap(facets[i--], facets[--j]);
  }
  facets.resize(j);
}

geo::Line STLFile::Limits() const {
  if (facets.empty()) return {Eigen::RowVector2d{0, 0}, {0, 0}};

  Eigen::RowVector3d minp = facets[0].edge[0];
  Eigen::RowVector3d maxp = minp;
  for (const auto& f : facets) {
    for (const auto& e : f.edge) {
      minp.x() = std::min(minp.x(), e.x());
      maxp.x() = std::max(maxp.x(), e.x());
      minp.y() = std::min(minp.y(), e.y());
      maxp.y() = std::max(maxp.y(), e.y());
    }
  }
  return {minp, maxp};
}

std::vector<geo::Line> STLFile::ToLines() const {
  // At this point, edges[:unseen] are line to be draw.
  ////////////// Start looking for hidden section.

  std::fenv_t env;
  std::feholdexcept(&env);  // Hope this deals with divide by zero issues.

  // Find intersections of lines
  std::vector<std::map<int, Eigen::RowVector3d>> hidden_by_edge = EdgeCrosses();

  //       edge hidden    from/to
  std::map<int, std::vector<Edge>> edge_hides_list;
  for (int fi = 0; fi < int(facets.size()); fi++) {
    const auto& f = facets[fi];
    const auto& edge_list = face_edge[fi];
    const auto e01 = f.edge[1] - f.edge[0];
    const auto e12 = f.edge[2] - f.edge[1];
    const auto e20 = f.edge[0] - f.edge[2];

    // For each visable edge, find the "first" and "last" point were the current
    // face hides the edge. Points on the edge are orderd by how close they are
    // to the "first" end.
    for (int v = 0; v < unseen; v++) {
      // An edge can't be hidden by the face that produced it.
      if ((v == edge_list[0]) || (v == edge_list[1]) || (v == edge_list[2])) {
        continue;
      }
      const auto& ev = edges[v];
      using PD = std::pair<Eigen::RowVector3d, double>;
      PD hidden_points[5];
      int idx = 0;
      // check if the ends of ev are hidden by f
      if (ev.first.dot(f.norm) < f.r) {
        auto v0 = ev.first - f.edge[0];
        auto v1 = ev.first - f.edge[1];
        auto v2 = ev.first - f.edge[2];

        auto x0 = e01.x() * v0.y() - e01.y() * v0.x();
        auto x1 = e12.x() * v1.y() - e12.y() * v1.x();
        auto x2 = e20.x() * v2.y() - e20.y() * v2.x();

        if ((x0 >= 0) && (x1 >= 0) && (x2 >= 0)) {
          hidden_points[idx++] = {ev.first, 0};
        }
      }

      if (ev.second.dot(f.norm) < f.r) {
        auto v0 = ev.second - f.edge[0];
        auto v1 = ev.second - f.edge[1];
        auto v2 = ev.second - f.edge[2];

        auto x0 = e01.x() * v0.y() - e01.y() * v0.x();
        auto x1 = e12.x() * v1.y() - e12.y() * v1.x();
        auto x2 = e20.x() * v2.y() - e20.y() * v2.x();

        if ((x0 >= 0) && (x1 >= 0) && (x2 >= 0)) {
          hidden_points[idx++] = {ev.second,
                                  (ev.first - ev.second).squaredNorm()};
        }
      }

      // Find places where this edge cosses this face
      const auto& edge_hidden = hidden_by_edge[v];
      for (int e : edge_list) {
        auto b = edge_hidden.find(e);
        if (b != edge_hidden.end()) {
          hidden_points[idx++] = {b->second,
                                  (ev.first - b->second).squaredNorm()};
        }
      }

      if (idx < 2) continue;

      std::sort(hidden_points, hidden_points + idx,
                [](const PD& l, const PD& r) { return l.second < r.second; });

      auto& p_min = hidden_points[0].first;
      auto& p_max = hidden_points[idx - 1].first;
      if ((p_min - p_max).squaredNorm() == 0) continue;
      CHECK(hidden_points[0].second <= hidden_points[idx - 1].second)
          << "Points out of order " << idx;
      edge_hides_list[v].emplace_back(p_min, p_max);
    }
  }

  // Find endpoints behind

  fesetenv(&env);  // return to the inital FP state

  LOG(INFO) << "Generate 2D lines";
  // Sort each set of hidden spans
  for (int i = 0; i < unseen; i++) {
    const auto& p = edges[i];
    auto& hides = edge_hides_list[i];
    std::sort(hides.begin(), hides.end(), [&](const Edge& l, const Edge& r) {
      return (p.first - l.first).squaredNorm() <
             (p.first - r.first).squaredNorm();
    });
  }

  std::vector<geo::Line> ret =
      GenerateLines(false, unseen, kMinLineLen, edges, edge_hides_list);
  LOG(INFO) << "Done rendering";
  return ret;
}

geo::point_set stl2ps::STLFile::Points() const {
  geo::point_set ret;
  for (const auto& f : facets)
    for (const auto& e : f.edge) ret.insert(e);
  return ret;
}

namespace {

struct EdgeOrder {
  bool operator()(const Edge& l, const Edge& r) const {
    if (geo::Order(l.first, r.first)) return true;
    if (geo::Order(r.first, l.first)) return false;
    return geo::Order(l.second, r.second);
  }
};

}  // namespace

void STLFile::Index() {
  edges.clear();
  face_edge.clear();
  face_edge.resize(facets.size(), {{-1, -1, -1}});
  std::vector<std::pair<int, int>> edge_face;  // index of faces by edge
  edge_face.reserve(facets.size() * 2);

  ////////////////////////
  // If this edge us a new one, update edges/edge_face/face_edge
  std::map<Edge, int, EdgeOrder> dd;
  const auto AddEdge = [&](const Eigen::RowVector3d& a,
                           const Eigen::RowVector3d& b, int f) {
    auto x =
        dd.emplace(geo::Order(a, b) ? Edge{a, b} : Edge{b, a}, edges.size());
    auto idx = x.first->second;

    if (x.second) {
      edges.push_back(x.first->first);  // fist sight
      edge_face.push_back({f, f});
    } else {
      edge_face[idx].second = f;
    }
  };

  for (uint32_t i = 0; i < facets.size(); i++) {
    const auto& f = facets[i];
    AddEdge(f.edge[0], f.edge[1], i);
    AddEdge(f.edge[1], f.edge[2], i);
    AddEdge(f.edge[2], f.edge[0], i);
  }

  unseen = edges.size();
  if (!show_invisible) {
    // Sort "invisible" edges to the end.
    for (int i = 0; i < unseen; i++) {
      const int a1 = edge_face[i].first, a2 = edge_face[i].second;
      if (a1 == a2) continue;  // leave "only once" edges alone.

      auto ang = facets[a1].norm.dot(facets[a2].norm);
      if (ang < min_ang) continue;

      // Swap ea and eb and update the other references.
      const int ea = i--, eb = --unseen;
      swap(edge_face[ea], edge_face[eb]);
      swap(edges[ea], edges[eb]);
    }
  }

  // Update the edge->index mapping
  dd.clear();
  for (uint32_t i = 0; i < edges.size(); i++) dd[edges[i]] = i;

  // Update the face to edge idx mapping.
  const auto UpdateEdge = [&](const Eigen::RowVector3d& a,
                              const Eigen::RowVector3d& b, int f, int e) {
    auto x = dd.find(geo::Order(a, b) ? Edge{a, b} : Edge{b, a});
    CHECK(x != dd.end()) << "edge not found";
    face_edge[f][e] = x->second;
  };

  for (uint32_t i = 0; i < facets.size(); i++) {
    const auto& f = facets[i];
    UpdateEdge(f.edge[0], f.edge[1], i, 0);
    UpdateEdge(f.edge[1], f.edge[2], i, 1);
    UpdateEdge(f.edge[2], f.edge[0], i, 2);
  }
}

//   edge hidden by edge @point
std::vector<std::map<int, Eigen::RowVector3d>> STLFile::EdgeCrosses() const {
  // Convert to a vector of maps:
  //   edge hidden by edge @point
  std::vector<std::map<int, Eigen::RowVector3d>> hidden_by_edge;
  hidden_by_edge.resize(unseen);
  if (show_hidden) return hidden_by_edge;
  std::vector<std::tuple<int, int, Eigen::RowVector3d>> hidden_by_list;
  hidden_by_list.reserve(edges.size());

  for (int v = 0; v < unseen; v++) {
    const auto& ev = edges[v];
    auto dv = ev.second - ev.first;
    for (int u = v + 1; u < int(edges.size()); u++) {
      const auto& eu = edges[u];
      auto du = eu.second - eu.first;

      // dv.x * j + -du.x * i = eu.first.x-ev.first.x;
      // dv.y * j + -du.y * i = eu.first.y-ev.first.y;
      // [[j][i]] = [[-du.y, du.x][-dv.y, dv.x]] * [[x][y]]/ det

      // Matrix inversion
      auto det = 1 / (du.x() * dv.y() - dv.x() * du.y());
      auto x = eu.first.x() - ev.first.x();
      auto y = eu.first.y() - ev.first.y();
      auto j = (-du.y() * x + du.x() * y) * det;
      auto i = (-dv.y() * x + dv.x() * y) * det;

      if (!(0 <= i && i <= 1 && 0 <= j && j <= 1)) {
        continue;
      }
      auto pu = eu.first + du * i;
      auto pv = ev.first + dv * j;

      if (pu.z() <= pv.z()) hidden_by_list.emplace_back(u, v, pu);
      if (pv.z() <= pu.z()) hidden_by_list.emplace_back(v, u, pv);
    }
  }
  for (const auto& h : hidden_by_list) {
    if (std::get<0>(h) >= unseen) continue;
    CHECK(hidden_by_edge[std::get<0>(h)]
              .emplace(std::get<1>(h), std::get<2>(h))
              .second)
        << "Duplicate found: " << std::get<0>(h) << "," << std::get<1>(h) << ","
        << std::get<2>(h);
  }
  return hidden_by_edge;
}

std::vector<geo::Line> GenerateLines(
    const bool show_hidden, const int unseen, double min_line_len,
    const std::vector<Edge>& edges,
    const std::map<int, std::vector<Edge>>& edge_hides) {
  static const std::vector<Edge> nil;
  ////////
  std::vector<geo::Line> ret;
  for (int i = 0; i < unseen; i++) {
    // The edge to show.
    const auto& p = edges[i];

    auto eh_it = edge_hides.find(i);
    // The hidden portions/
    const auto& hides =
        (!show_hidden && eh_it != edge_hides.end()) ? eh_it->second : nil;

    auto from = p.first;
    double d = 0;
    for (const auto& hid : hides) {
      // If the next hidden section is after `from`
      // draw the line [from, hid.first].
      auto hid_d = (p.first - hid.first).squaredNorm();
      if (d < hid_d) {
        ret.emplace_back(from, hid.first);
      }

      // if `from` is befor the end of the hidden section,
      // Move `from` to the end of it.
      auto n = (p.first - hid.second).squaredNorm();
      if (d < n) {
        d = n;
        from = hid.second;
      }
      CHECK(hid_d <= n) << "Hidden section reversed: " << hid_d << ", " << n;
    }
    if ((from - p.second).squaredNorm() > min_line_len * min_line_len) {
      ret.emplace_back(from, p.second);
    }
  }
  return ret;
}

////////////////////////////////

}  // namespace stl2ps
