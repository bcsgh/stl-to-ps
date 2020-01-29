workspace(name = "stl_to_ps")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")

# Note:
# local_repository(name = "...", path = "/home/...")

#############################################
# needed by com_github_glog_glog
git_repository(
    name = "com_github_gflags_gflags",
    remote = "git://github.com/gflags/gflags.git",
    tag = "v2.2.2",  # current as of 2020/1/25
)

#############################################
git_repository(
    name = "com_github_glog_glog",
    remote = "git://github.com/google/glog.git",
    tag = "v0.4.0",
)

#############################################
git_repository(
    name = "com_googlesource_code_re2",
    remote = "git://github.com/google/re2.git",
    tag = "2020-01-01",  # current as of 2020/1/25
)

#############################################
git_repository(
    name = "com_google_googletest",
    remote = "git://github.com/google/googletest.git",
    tag = "release-1.10.0",  # current as of 2020/1/25
)

#############################################
git_repository(
    name = "com_google_absl",
    commit = "44427702614d7b86b064ba06a390f5eb2f85dbf6",  # current as of 2020/1/25
    remote = "git://github.com/abseil/abseil-cpp.git",
)

#############################################
new_git_repository(
    name = "eigen",
    build_file = "@//:BUILD.eigen",
    remote = "git://github.com/eigenteam/eigen-git-mirror.git",
    tag = "3.3.7",  # current as of 2020/1/25
)

#############################################
git_repository(
    name = "bazel_rules",
    commit = "327b56205204c2517ed50a3fec64dc5841479a64",  # current as of 2020/1/28
    remote = "git://github.com/bcsgh/bazel_rules.git",
)
