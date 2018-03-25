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

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace stl2ps {
namespace {

TEST(Center, Empty) {
  std::vector<Eigen::RowVector2d> points;
  Eigen::RowVector2d center;
  double r = 0;

  EXPECT_FALSE(stl2ps::FindCircle(points, &center, &r));
}

TEST(Center, Exact) {
  std::vector<Eigen::RowVector2d> points{
      (Eigen::RowVector2d() << 0, 1).finished(),
      (Eigen::RowVector2d() << 1, 2).finished(),
      (Eigen::RowVector2d() << 2, 1).finished(),
  };
  Eigen::RowVector2d center;
  double r = 0;

  ASSERT_TRUE(stl2ps::FindCircle(points, &center, &r));

  EXPECT_EQ(center.x(), 1);
  EXPECT_EQ(center.y(), 1);
  EXPECT_EQ(r, 1);
}

// Generate a set of points that approximate a circle
std::vector<Eigen::RowVector2d> Points() {
  std::vector<Eigen::RowVector2d> points;
  for (float t = 0; t < 3.14 * 2; t += .1) {
    float x = 20 + sin(t) * (15 + sin(t * 5));
    float y = 30 + cos(t) * (15 + sin(t * 5));
    points.push_back((Eigen::RowVector2d() << x, y).finished());
  };
  return points;
}

TEST(Center, Inexact) {
  std::vector<Eigen::RowVector2d> points = Points();
  Eigen::RowVector2d center;
  double r = 0;

  ASSERT_TRUE(stl2ps::FindCircle(points, &center, &r));

  EXPECT_THAT(center.x(), testing::DoubleNear(20, 0.00003));
  EXPECT_THAT(center.y(), testing::DoubleNear(30, 0.002));
  EXPECT_THAT(r, testing::DoubleNear(15, 0.02));
}

TEST(Center, PositionInvariant) {
  std::vector<Eigen::RowVector2d> points = Points();
  for (auto& p : points) {  // Move things a long ways out.
    p.x() += 234;
    p.y() += 567;
  }
  Eigen::RowVector2d center;
  double r = 0;

  ASSERT_TRUE(stl2ps::FindCircle(points, &center, &r));

  // Same as base case but with the same offset added
  EXPECT_THAT(center.x(), testing::DoubleNear(20 + 234, 0.00003));
  EXPECT_THAT(center.y(), testing::DoubleNear(30 + 567, 0.002));
  EXPECT_THAT(r, testing::DoubleNear(15, 0.02));
}

TEST(Center, ScaleInvariant) {
  std::vector<Eigen::RowVector2d> points = Points();
  for (auto& p : points) {  // Make things a lot bigger.
    p.x() *= 234;
    p.y() *= 234;
  }
  Eigen::RowVector2d center;
  double r = 0;

  ASSERT_TRUE(stl2ps::FindCircle(points, &center, &r));

  // Everything gets bigger (including the errors) by the same factor
  EXPECT_THAT(center.x(), testing::DoubleNear(20 * 234, 234 * 0.00003));
  EXPECT_THAT(center.y(), testing::DoubleNear(30 * 234, 234 * 0.002));
  EXPECT_THAT(r, testing::DoubleNear(15 * 234, 234 * 0.02));
}

}  // namespace
}  // namespace stl2ps