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

#ifndef STL_TO_PS_AST_H_
#define STL_TO_PS_AST_H_

#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "absl/base/attributes.h"
#include "glog/logging.h"
#include "stl-to-ps/eigen_wrap.h"
#include "stl-to-ps/geo.h"

namespace stl2ps {
// Generaric location type.
struct Loc {
  template <class L>
  Loc(L l) : filename(*l.begin.filename), line(l.begin.line) {}
  Loc() = default;

  friend ::std::ostream& operator<<(::std::ostream& os, const Loc& l) {
    return os << l.filename << ":" << l.line;
  }

  std::string filename = "??";
  int line = 0;
};

// An AST node with a location that can be logged
class NodeI {
 public:
  explicit NodeI(Loc loc) : loc_(std::move(loc)) {}

  const Loc& location() const { return loc_; }
  const std::string& source_file() const { return loc_.filename; }
  int source_line() const { return loc_.line; }

 private:
  Loc loc_;
};

/////////////////////////////
// A helper for creating error messages.
class ErrorMessage {
 public:
  ErrorMessage(const char* file, int line, const NodeI& n)
      : ErrorMessage(file, line, n.location()) {}
  ErrorMessage(const char* file, int line, const Loc& l) {
#ifndef NDEBUG
    get() << "(" << file << ":" << line << ") ";
#endif  // NDEBUG
    get() << l << ": ";
  }
  ~ErrorMessage() { get() << std::endl; }

  std::ostream& get() { return std::cerr; }
};

#define SYM_ERROR(x) ErrorMessage(__FILE__, __LINE__, (x)).get()

typedef std::pair<int, int> Scale;

class Point : public NodeI {
 public:
  using NodeI::NodeI;
  virtual ~Point() = default;

  virtual void Rotate(Eigen::Matrix3d rot) = 0;
  virtual bool Invoke(const geo::point_set& ps, Eigen::RowVector3d* ret) = 0;
};

class Val final : public Point {
 public:
  Val(Eigen::RowVector3d p, Loc loc) : Point(loc), p_(p) {}

  void Rotate(Eigen::Matrix3d rot) override { p_ = p_ * rot; }
  bool Invoke(const geo::point_set& ps, Eigen::RowVector3d* ret) override {
    *ret = p_;
    return true;
  }

 private:
  Eigen::RowVector3d p_;
};

namespace point_impl {
// Find the `count` closets points from `ps` to `to`.
std::vector<Eigen::RowVector2d> Closest(
    const std::vector<Eigen::RowVector2d>& ps, int count,
    Eigen::RowVector2d to);

// return all points in `ps` that are between `min` and `max` from `to`.
std::vector<Eigen::RowVector2d> Between(
    const std::vector<Eigen::RowVector2d>& ps, double min, double max,
    Eigen::RowVector2d to);
}  // namespace point_impl

class PointFunc final : public Point {
 public:
  PointFunc(std::unique_ptr<std::string> n, std::unique_ptr<Point> p, Loc loc)
      : Point(loc), name_(std::move(*n)), p_(std::move(p)) {}

  void Rotate(Eigen::Matrix3d rot) override { p_->Rotate(rot); }
  bool Invoke(const geo::point_set& ps, Eigen::RowVector3d* ret) override;

 private:
  bool Near(const geo::point_set& ps, Eigen::RowVector3d* ret);
  bool Center(const geo::point_set& ps, Eigen::RowVector3d* ret);

  std::string name_;
  std::unique_ptr<Point> p_;
};

/////////////////////////////////
template <class T>
class MetaValue;
template <class T>
class MetaRef;

class Meta : public NodeI {
 public:
  virtual ~Meta() = default;

  template <class T, class N, class V>
  static typename std::enable_if<!std::is_abstract<T>::value,
                                 std::unique_ptr<MetaValue<T>>>::type
  New(N name, V value, Loc loc) {
    auto ret = std::unique_ptr<MetaValue<T>>{
        new MetaValue<T>(std::move(loc), Take<std::string>(std::move(name)),
                         Take<T>(std::move(value)))};
    return ret;
  }

  template <class T, class N>
  static typename std::enable_if<std::is_abstract<T>::value,
                                 std::unique_ptr<MetaRef<T>>>::type
  New(N name, T* t, Loc loc) {
    auto ret = std::unique_ptr<MetaRef<T>>{
        new MetaRef<T>(std::move(loc), Take<std::string>(std::move(name)), t)};
    return ret;
  }

  virtual std::string type_name() = 0;
  template <class T>
  ABSL_MUST_USE_RESULT bool As(T* t) const;
  template <class T>
  T* get();

  std::string name;

 protected:
  Meta(Loc l, std::string n) : NodeI(std::move(l)), name(std::move(n)) {}

  template <class T>
  static std::string Demangle() {
    return UnsafeDemangle(typeid(T).name());
  }

 private:
  template <class T>
  static T Take(T t) {
    return std::move(t);
  }

  template <class T>
  static T Take(T* t) {
    return std::move(*std::unique_ptr<T>(t));
  }

  // Demanglers are hard to make tolerant to malicious inputs,
  // so restrict this to an API the just does names of built in types.
  static std::string UnsafeDemangle(const char* name);
};

////////////////////////////////
template <class T>
class MetaValue final : public Meta {
 private:
  friend class Meta;
  MetaValue(Loc loc, std::string n, T v)
      : Meta(std::move(loc), std::move(n)), value(std::move(v)) {}
  MetaValue(Loc loc, std::unique_ptr<std::string> n, T v)
      : MetaValue(std::move(loc), std::move(*n), std::move(v)) {}
  MetaValue(Loc loc, std::unique_ptr<std::string> n, std::unique_ptr<T> v)
      : MetaValue(std::move(loc), std::move(*n), std::move(*v)) {}

  T* get_impl() { return &value; }

 public:
  std::string type_name() override { return Demangle<T>(); }

  T value;
};

////////////////////////////////
template <class T>
class MetaRef final : public Meta {
 private:
  friend class Meta;
  MetaRef(Loc loc, std::string n, T* v)
      : Meta(std::move(loc), std::move(n)), value(v) {}

  T* get_impl() { return value.get(); }

 public:
  std::string type_name() override { return Demangle<T>(); }

  std::unique_ptr<T> value;
};

template <class T>
bool Meta::As(T* t) const {
  static_assert(!std::is_abstract<T>::value,
                "Can't use Meta::As for abstract types");
  const auto* typed = dynamic_cast<const MetaValue<T>*>(this);
  if (typed == nullptr) return false;
  *t = typed->value;
  return true;
}

template <class T>
T* Meta::get() {
  using MT = typename std::conditional<std::is_abstract<T>::value, MetaRef<T>,
                                       MetaValue<T>>::type;
  auto typed = dynamic_cast<MT*>(this);
  if (typed == nullptr) {
    LOG(INFO) << typeid(*this).name() << " != " << typeid(MT).name();
    return nullptr;
  }
  return typed->get_impl();
}

/////////////////////////////////
struct Model final : public NodeI {
  Model(std::unique_ptr<std::string> n, std::unique_ptr<std::string> s, Loc l)
      : Model(std::move(*n), std::move(*s), std::move(l)) {}
  Model(std::string n, std::string s, Loc l)
      : NodeI(std::move(l)), name(std::move(n)), source(std::move(s)) {}
  std::string name;
  std::string source;
};

/////////////////////////////////
struct VisitDrawable;
using MetaList = std::vector<std::unique_ptr<Meta>>;

/////////////////////////////////
struct Drawable : public NodeI {
  using NodeI::NodeI;
  Drawable(const Drawable&) = delete;
  Drawable(Drawable&&) = default;
  Drawable(Loc loc, MetaList l)
      : NodeI(std::move(loc)), meta_list(std::move(l)){};
  virtual ~Drawable() = default;

  MetaList meta_list;

  ABSL_MUST_USE_RESULT bool VisitNode(VisitDrawable* v) { return Visit(v); }

 private:
  ABSL_MUST_USE_RESULT virtual bool Visit(VisitDrawable*) = 0;
};

struct BaseDim : public Drawable {
  using Drawable::Drawable;
  BaseDim(const BaseDim&) = delete;
  BaseDim(BaseDim&&) = default;
  virtual ~BaseDim() = default;
};

struct Angle final : public BaseDim {
  using BaseDim::BaseDim;
  bool Visit(VisitDrawable*) override;
};

struct Dim final : public BaseDim {
  using BaseDim::BaseDim;
  bool Visit(VisitDrawable* v) override;
};

struct Dia final : public BaseDim {
  using BaseDim::BaseDim;
  bool Visit(VisitDrawable*) override;
};

struct Rad final : public BaseDim {
  using BaseDim::BaseDim;
  bool Visit(VisitDrawable*) override;
};

struct DrawList {
  MetaList meta_list;
  std::vector<std::unique_ptr<BaseDim>> dims;
};

struct Draw final : public Drawable {
  Draw(Loc l, std::unique_ptr<std::string> n, std::unique_ptr<DrawList> dl)
      : Drawable(std::move(l), std::move(dl->meta_list)),
        name(std::move(*n)),
        dims(std::move(dl->dims)) {}

  bool Visit(VisitDrawable* v) override;

  std::string name;
  std::vector<std::unique_ptr<BaseDim>> dims;
};

struct Text final : public Drawable {
  Text(Loc l, std::unique_ptr<std::string> t, std::unique_ptr<MetaList> ml)
      : Drawable(std::move(l), std::move(*ml)), text(std::move(*t)) {}

  bool Visit(VisitDrawable* v) override;

  std::string text;
};

struct VisitDrawable {
  ABSL_MUST_USE_RESULT virtual bool operator()(const Angle&) = 0;
  ABSL_MUST_USE_RESULT virtual bool operator()(const Dia&) = 0;
  ABSL_MUST_USE_RESULT virtual bool operator()(const Dim&) = 0;
  ABSL_MUST_USE_RESULT virtual bool operator()(const Draw&) = 0;
  ABSL_MUST_USE_RESULT virtual bool operator()(const Rad&) = 0;
  ABSL_MUST_USE_RESULT virtual bool operator()(const Text&) = 0;

  // Only exact matches are allowed. Suppress all conversions.
  template <class T>
  bool operator()(const T&) = delete;
};

/////////////////////////////////
struct PageParts {
  MetaList meta;
  std::vector<std::unique_ptr<Drawable>> draws;
};

struct Page final : public NodeI {
  Page(Loc l, std::unique_ptr<PageParts> p)
      : NodeI(std::move(l)),
        meta(std::move(p->meta)),
        draws(std::move(p->draws)) {}

  MetaList meta;
  std::vector<std::unique_ptr<Drawable>> draws;
};

/////////////////////////////////
struct Document {
  void Add(std::unique_ptr<Model>);
  void Add(std::unique_ptr<Page>);

  std::vector<Model> models;
  std::vector<Page> pages;
};

// Take a quoted escaped string and return it's unescaped content.
std::string* NewQuote(std::string);

bool GetMatrixByName(const std::string&, Eigen::Matrix3d*);
Eigen::Matrix3d GetMatrixByAng(const Eigen::RowVector2d&);
}  // namespace stl2ps

#endif  // STL_TO_PS_AST_H_
