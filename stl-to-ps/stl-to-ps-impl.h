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

#ifndef STL2PS_IMPL_
#define STL2PS_IMPL_

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "stl-to-ps/ast.h"
#include "stl-to-ps/ps.h"
#include "stl-to-ps/stl.h"

namespace stl2ps {

// Implementation details.
struct OutputPage {
  void AddHeader(const std::string &name, int page, int of);

  std::vector<geo::Line> lines;
  std::vector<geo::Arc> arcs;
  std::vector<ps::Text> text;
};

class DrawToPage : public VisitDrawable {
 public:
  DrawToPage(const std::map<std::string, std::unique_ptr<STLFile>> &f)
      : stl_files(f){};
  void set_current_page(OutputPage *c) { current_page = c; }

  bool operator()(const Angle &) override;
  bool operator()(const Dia &) override;
  bool operator()(const Dim &) override;
  bool operator()(const Draw &) override;
  bool operator()(const Rad &) override;
  bool operator()(const Text &) override;

  void AddArcs(const std::vector<geo::Arc> &arcs);
  ABSL_MUST_USE_RESULT bool AddDims(
      const STLFile &file, const std::vector<std::unique_ptr<BaseDim>> &dims,
      Eigen::Matrix3d rotation);
  void AddLines(const std::vector<geo::Line> &lines);
  void AddText(const std::vector<ps::Text> &text);

 private:
  friend class DrawToPageTests;

  bool GetCenter(const BaseDim& dia, absl::string_view name,
                 Eigen::RowVector2d* at, Eigen::RowVector2d* dir,
                 Eigen::RowVector2d* ret_center, double* ret_rad);
  bool GetRotated(stl2ps::Meta *p_in, Eigen::RowVector3d *p_out);

  bool RenderDia(Eigen::RowVector2d center, Eigen::RowVector2d at,
                 Eigen::RowVector2d dir, double r);
  bool RenderRad(Eigen::RowVector2d center, Eigen::RowVector2d at,
                 Eigen::RowVector2d dir, double r);

  const std::map<std::string, std::unique_ptr<STLFile>> &stl_files;
  OutputPage *current_page = nullptr;

  // Working set for use by operator()'s
  geo::point_set points_;
  Eigen::Matrix3d rotation_;

  struct {
    float scale = 0;
    Eigen::RowVector2d d = {0, 0};
  } proj;
};

void RenderPages(const std::string &src, const std::vector<OutputPage> &pages,
                 std::ostream &out);

bool LoadFiles(const Document &doc,
               std::map<std::string, std::unique_ptr<STLFile>> *stl_files);

bool GeneratePages(
    const std::map<std::string, std::unique_ptr<STLFile>> &stl_files,
    const Document &doc, std::vector<OutputPage> *pages);

bool ScriptToPS(const std::string &src, const Document &doc, std::ostream &out);

}  // namespace stl2ps

#endif  // STL2PS_IMPL_
