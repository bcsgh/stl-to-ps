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

#include "stl-to-ps/ps.h"

#include <vector>

#include "absl/strings/substitute.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "gflags/gflags.h"
#include "stl-to-ps/geo.h"

DEFINE_string(name, "", "the name of the image");
DEFINE_bool(timestamp, true, "Include the date/time on each page");

namespace ps {
namespace {
std::string Now() {
  const static auto ret = [] {
    absl::TimeZone tz;
    absl::LoadTimeZone("localtime", &tz);

    return absl::FormatTime("%Y-%m-%d %I:%M:%S", absl::Now(), tz);
  }();
  return ret;
}
}  // namespace

void DocumentHeader(const std::string& src, std::ostream& out) {
  out << absl::Substitute(R"(% generated by stl-to-ps
% Name: $0
% Source: $1
% Time: $2
%
% File Header %%%%%%
/FontSize { $3 } def  % a hack to allow reusing the font size

/CenterText {
  dup stringwidth
  pop     % throw away the "height"
  -2 div  % half and negate

  currentfont begin FontBBox end  % - y_min - y_max
  4 1 roll pop pop pop 2 div      % midline in "units"
  -1600 div FontSize mul          % convert to em

  rmoveto % offset by that
} def

/Courier findfont
FontSize scalefont setfont
% End File Header %%%%%%
)",
                          FLAGS_name, src, Now(), kFontSize)
      << std::endl;
}

void PageHeader(int num, int pages, std::ostream& out) {
  out << absl::Substitute(R"(
%%%%%%%%% Start of page $0 of $1
<< /PageSize [792 612] >> setpagedevice % letter/landscape

.1 setlinewidth
% draw border
newpath 5 5 moveto 5 607 lineto 787 607 lineto 787 5 lineto 5 5 lineto stroke
)",
                          num, pages)
      << std::flush;
  if (FLAGS_timestamp) {
    Text t;
    t.at = {7, 7};
    t.str = Now();
    TextToPs({t}, out);
  }
}

void LinesToPs(const std::vector<geo::Line>& lines, std::ostream& out) {
  for (const auto& l : lines) {
    out << absl::Substitute("newpath $0 $1 moveto $2 $3 lineto stroke\n",  //
                            l.o.x(), l.o.y(), l.t.x(), l.t.y());
  }
}

void ArcToPs(const std::vector<geo::Arc>& arcs, std::ostream& out) {
  for (const auto& a : arcs) {
    out << absl::Substitute("newpath $0 $1 $2 $3 $4 arc stroke\n",  //
                            a.center.x(), a.center.y(),             //
                            a.r, a.start * 180 / geo::PI,
                            a.end * 180 / geo::PI);
  }
}

void TextToPs(const std::vector<Text>& text, std::ostream& out) {
  for (const auto t : text) {
    std::string s;
    if (t.raw) {
      s = t.str;
    } else {
      s.reserve(t.str.size());
      for (char c : t.str) {
        switch (c) {
          default:
            s.push_back(c);
            break;

          case '(':
          case ')':
          case '\\':
            s.push_back('\\');
            s.push_back(c);
            break;
        }
      }
    }
    out << absl::Substitute(
        (t.center ? "newpath $0 $1 moveto ($2) CenterText show\n"
                  : "newpath $0 $1 moveto ($2) show\n"),
        t.at.x(), t.at.y(), s);
  }
}

void PageFooter(std::ostream& out) {
  out << R"(
showpage %%%%%%%%%
)" << std::flush;
}

}  // namespace ps
