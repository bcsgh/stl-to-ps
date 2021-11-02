workspace(name = "stl_to_ps")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

# Note:
# local_repository(name = "...", path = "/home/...")

#############################################
# needed by com_github_glog_glog
git_repository(
    name = "com_github_gflags_gflags",
    commit = "827c769e5fc98e0f2a34c47cef953cc6328abced",  # current as of 2021/02/17
    remote = "https://github.com/gflags/gflags.git",
    shallow_since = "1604052972 +0000",
)

#############################################
git_repository(
    name = "com_github_glog_glog",
    commit = "17e7679fd9beb95277ccd0708056ba85363f892b",  # current as of 2021/11/01
    remote = "https://github.com/google/glog.git",
    shallow_since = "1635542045 +0200",
)

#############################################
git_repository(
    name = "com_googlesource_code_re2",
    commit = "0dade9ff39bb6276f18dd6d4bc12d3c20479ee24",  # current as of 2021/11/01
    remote = "https://github.com/google/re2.git",
    shallow_since = "1635432154 +0000",
)

#############################################
git_repository(
    name = "com_google_googletest",
    commit = "16f637fbf4ffc3f7a01fa4eceb7906634565242f",  # current as of 2021/11/01
    remote = "https://github.com/google/googletest.git",
    shallow_since = "1634142500 -0400",
)

#############################################
git_repository(
    name = "com_google_absl",
    commit = "022527c50e0e2bc937f9fa3c516e3e36cbba0845",  # current as of 2021/11/01
    remote = "https://github.com/abseil/abseil-cpp.git",
    shallow_since = "1635537114 -0400",
)

#############################################
git_repository(
    name = "bazel_rules",
    commit = "591a088101314c23c940cc30e4e4081d1338be29",  # current as of 2021/11/01
    remote = "https://github.com/bcsgh/bazel_rules.git",
    shallow_since = "1635825075 -0700",
)

load("@bazel_rules//repositories:repositories.bzl", "eigen")

#############################################
eigen()
