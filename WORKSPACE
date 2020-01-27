workspace(name = "stl_to_ps")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")

# Note:
# local_repository(name = "...", path = "/home/...")

#############################################
git_repository(
    name = "com_github_gflags_gflags",
    tag = "v2.2.2",  # current as of 2020/1/25
    remote = "git://github.com/gflags/gflags.git",
)

#############################################
git_repository(
    name = "com_github_glog_glog",
    tag = "v0.4.0",
    remote = "git://github.com/google/glog.git",
)

#############################################
git_repository(
    name = "com_googlesource_code_re2",
    tag = "2020-01-01",  # current as of 2020/1/25
    remote = "git://github.com/google/re2.git",
)

#############################################
git_repository(
    name = "com_google_googletest",
    tag = "release-1.10.0",  # current as of 2020/1/25
    remote = "git://github.com/google/googletest.git",
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
    tag = "3.3.7",  # current as of 2020/1/25
    remote = "git://github.com/eigenteam/eigen-git-mirror.git",
    build_file = "@//:BUILD.eigen",
)

#############################################
git_repository(
    name = "bazel_rules",
    commit = "078429ead0c7ab8cab076221f5d444bdfa1be4bb",  # current as of 2020/1/25
    remote = "git://github.com/bcsgh/bazel_rules.git",
)
