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

#ifndef STL_TO_PS_GEO_H_
#define STL_TO_PS_GEO_H_

#include <cmath>
#include <set>

#include "stl-to-ps/eigen_wrap.h"

namespace geo {
constexpr double PI = 3.1415926535897932;

////////////////////////////
inline bool Order(const Eigen::RowVector3d& l, const Eigen::RowVector3d& r) {
  return (l.x() != r.x())
             ? (l.x() < r.x())
             : (l.y() != r.y()) ? (l.y() < r.y()) : (l.z() < r.z());
}
struct PointOrder {
  bool operator()(const Eigen::RowVector3d& l,
                  const Eigen::RowVector3d& r) const {
    return Order(l, r);
  }
};
using point_set = std::set<Eigen::RowVector3d, PointOrder>;

Eigen::Matrix3d Rotate(double ang, double azm);

////////////////////////////
struct Line {
  Eigen::RowVector2d o, t;

  Line(Eigen::RowVector2d o_, Eigen::RowVector2d t_) : o(o_), t(t_) {}
  Line(Eigen::RowVector3d o_, Eigen::RowVector3d t_)
      : Line(Eigen::RowVector2d{o_.x(), o_.y()}, {t_.x(), t_.y()}) {}
  Line() = default;
};

struct Arc {
  Eigen::RowVector2d center;
  double r, start, end;

  Arc(Eigen::RowVector2d c, double r_, double d, double e)
      : center(c), r(r_), start(d), end(e) {}
  Arc(Eigen::RowVector3d a, double r, Eigen::RowVector3d s,
      Eigen::RowVector3d e)
      : Arc(Eigen::RowVector2d{a.x(), a.y()}, r, std::atan2(s.y(), s.x()),
            std::atan2(e.y(), e.x())) {}
  Arc() = default;
};

bool operator<(const Line& l, const Line& r);
bool operator==(const Line& l, const Line& r);

////////////////////////////

///////////////////////// Constants
namespace point {
const Eigen::RowVector3d zero{0, 0, 0};
const Eigen::RowVector3d x{1, 0, 0};
const Eigen::RowVector3d y{0, 1, 0};
const Eigen::RowVector3d z{0, 0, 1};
}  // namespace point

namespace matrix {
inline Eigen::Matrix3d ByRow(Eigen::RowVector3d p0, Eigen::RowVector3d p1,
                             Eigen::RowVector3d p2) {
  return (Eigen::Matrix3d{} << p0, p1, p2).finished();
}
const Eigen::Matrix3d XP = ByRow(point::z, point::x, point::y);
const Eigen::Matrix3d XN = ByRow(-point::z, -point::x, point::y);
const Eigen::Matrix3d YP = ByRow(-point::x, point::z, point::y);
const Eigen::Matrix3d YN = ByRow(point::x, -point::z, point::y);
const Eigen::Matrix3d ZP = ByRow(point::x, point::y, point::z);
const Eigen::Matrix3d ZN = ByRow(point::x, -point::y, -point::z);
const Eigen::Matrix3d I = ZP;
}  // namespace matrix

//////////////////////////// Inline functions

inline bool operator<(const Line& l, const Line& r) {
  if (l.o.x() != r.o.x()) return (l.o.x() < r.o.x());
  if (l.o.y() != r.o.y()) return (l.o.y() < r.o.y());
  return (l.t.x() != r.t.x()) ? (l.t.x() < r.t.x()) : (l.t.y() < r.t.y());
}

inline bool operator==(const Line& l, const Line& r) {
  return (l.o == r.o) && (l.t == r.t);
}

}  // namespace geo

#endif  // STL_TO_PS_GEO_H_
