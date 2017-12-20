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

#ifndef STL_TO_PS_PS_H_
#define STL_TO_PS_PS_H_

#include <iostream>
#include <string>
#include <vector>

#include "stl-to-ps/common.h"

namespace ps {

constexpr int kFontSize = 8;

struct Text {
  Text() = default;
  Text(float x_, float y_, std::string s) : x(x_), y(y_), str(std::move(s)) {}

  float x;
  float y;
  std::string str;
  bool center = false;
};

void DocumentHeader(const std::string &src, std::ostream &out);

void PageHeader(int num, int pages, std::ostream &out);

void LinesToPs(const std::vector<geo::Line> &lines, std::ostream &out);
void ArcToPs(const std::vector<geo::Arc> &arcs, std::ostream &out);
void TextToPs(const std::vector<Text> &lines, std::ostream &out);

void PageFooter(std::ostream &out);

}  // namespace ps

#endif  // STL_TO_PS_PS_H_
