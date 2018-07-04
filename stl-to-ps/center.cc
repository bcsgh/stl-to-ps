// Copyright (c) 2018, Benjamin Shropshire,
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

#include "stl-to-ps/center.h"

#include <vector>

#include "absl/strings/str_split.h"
#include "stl-to-ps/eigen_wrap.h"

namespace stl2ps {

bool FindCircle(const std::vector<Eigen::RowVector2d>& points,
                Eigen::RowVector2d* center, double* rad) {
  if (points.size() < 3) return false;
  // y = xb + e
  // b = (Xt X)^-1 Xt Y

  // (X - Xc)^2 + (Y - Yc)^2 - R^2 = 0
  // X^2 - 2*X*Xc + Xc^2 + Y^2 - 2*Y*Yc + Yc^2 - R^2 = 0
  // X*(Xc) + Y*(Yc) + 1/2*(R^2 - Xc^2 - Yc^2) = (X^2 + Y^2)/2

  Eigen::Matrix<double, Eigen::Dynamic, 3> X(points.size(), 3);
  Eigen::Matrix<double, Eigen::Dynamic, 1> Y(points.size(), 1);

  for (size_t i = 0; i < points.size(); i++) {
    auto p = points[i];
    X(i, 0) = p.x();
    X(i, 1) = p.y();
    X(i, 2) = 0.5;
    Y[i] = p.dot(p) / 2;
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> Xt = X.transpose();
  Eigen::Vector3d res = (Xt * X).inverse() * Xt * Y;

  center->x() = res.x();
  center->y() = res.y();
  *rad = std::sqrt(res.z() + std::pow(res.x(), 2) + std::pow(res.y(), 2));
  return true;
}

}  // namespace stl2ps
