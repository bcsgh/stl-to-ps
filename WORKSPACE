workspace(name = "stl_to_ps")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

#############################################
# Bazel Skylib.
http_archive(
    name = "bazel_skylib",
    urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/1.2.1/bazel-skylib-1.2.1.tar.gz"],
    sha256 = "f7be3474d42aae265405a592bb7da8e171919d74c16f082a5457840f06054728",
)

# Note:
# local_repository(name = "...", path = "/home/...")

#############################################
# needed by com_github_glog_glog
git_repository(
    name = "com_github_gflags_gflags",
    commit = "a738fdf9338412f83ab3f26f31ac11ed3f3ec4bd",  # current as of 2022/10/25
    remote = "https://github.com/gflags/gflags.git",
    shallow_since = "1658955969 +0100",
)

#############################################
git_repository(
    name = "com_github_glog_glog",
    commit = "05fbc65278db1aa545ca5cb743c31bc717a48d0f",  # current as of 2022/10/25
    remote = "https://github.com/google/glog.git",
    shallow_since = "1663772402 +0200",
)

#############################################
git_repository(
    name = "com_googlesource_code_re2",
    commit = "7a65faf439295e941baa6640a717d89c1f13e9cd",  # current as of 2022/10/27
    remote = "https://github.com/google/re2.git",
    shallow_since = "1666860568 +0000",
)

#############################################
git_repository(
    name = "com_google_googletest",
    commit = "3026483ae575e2de942db5e760cf95e973308dd5",  # current as of 2022/10/25
    remote = "https://github.com/google/googletest.git",
    shallow_since = "1666712359 -0700",
)

#############################################
git_repository(
    name = "com_google_absl",
    commit = "827940038258b35a29279d8c65b4b4ca0a676f8d",  # current as of 2022/10/27
    remote = "https://github.com/abseil/abseil-cpp.git",
    shallow_since = "1666903548 -0700",
)

#############################################
git_repository(
    name = "bazel_rules",
    commit = "44423acf159eade80e8daee0db8ad81246052c91",  # current as of 2022/10/27
    remote = "https://github.com/bcsgh/bazel_rules.git",
    shallow_since = "1666916861 -0700"
)

load("@bazel_rules//repositories:repositories.bzl", "eigen")

#############################################
eigen()
