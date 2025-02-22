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

#ifndef STL_TO_PS_STL_H_
#define STL_TO_PS_STL_H_

#include <array>
#include <cfenv>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "stl-to-ps/eigen_wrap.h"
#include "stl-to-ps/geo.h"

namespace stl2ps {

////////////////////////////
////////////////////////////
typedef std::pair<Eigen::RowVector3d, Eigen::RowVector3d> Edge;
std::ostream& operator<<(std::ostream& o, const Edge& e);

////////////////////////////
struct Facet {
  Eigen::RowVector3d norm;
  Eigen::RowVector3d edge[3];
  double r;  // v dot norm == r -> v is in the plain of *this.

  Facet(const Eigen::RowVector3d& n, const std::vector<Eigen::RowVector3d>& e);
  Facet() = default;
  Facet(const Facet&) = default;
};

////////////////////////////
class STLFile {
 public:
  static std::unique_ptr<STLFile> Make(std::istream&);

  STLFile Rotate(const Eigen::Matrix3d& m) const;
  void CullBackFace();
  geo::Line Limits() const;
  std::vector<geo::Line> ToLines() const;

  // Get the set of vertexes from all faces.
  geo::point_set Points() const;

 private:
  void Index();
  std::vector<std::map<int, Eigen::RowVector3d>> EdgeCrosses() const;

  std::vector<Facet> facets;
  std::vector<Edge> edges;                    // All edges w/o dupes
  std::vector<std::array<int, 3>> face_edge;  // index of edges by face
  int unseen = 0;                             // Index of first "unseen" edge.
};
////////////////////////////

std::vector<geo::Line> GenerateLines(
    bool show_hidden, int unseen, double min_line_len,
    const std::vector<Edge>& edges,
    const std::map<int, std::vector<Edge>>& face_hides);

}  // namespace stl2ps

#endif  // STL_TO_PS_STL_H_
