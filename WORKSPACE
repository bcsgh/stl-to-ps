workspace(name = "stl_to_ps")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")

# Note:
# local_repository(name = "...", path = "/home/...")

#############################################
# needed by com_github_glog_glog
git_repository(
    name = "com_github_gflags_gflags",
    remote = "git://github.com/gflags/gflags.git",
    commit = "827c769e5fc98e0f2a34c47cef953cc6328abced",  # current as of 2021/02/17
    shallow_since = "1604052972 +0000",
)

#############################################
git_repository(
    name = "com_github_glog_glog",
    remote = "git://github.com/google/glog.git",
    commit = "e370cd51661892cb3bd5ba80541d0739c0d219b4",  # current as of 2021/02/17
    shallow_since = "1613215412 +0100",
)

#############################################
git_repository(
    name = "com_googlesource_code_re2",
    remote = "git://github.com/google/re2.git",
    commit = "7107ebc4fbf7205151d8d2a57b2fc6e7853125d4",  # current as of 2021/02/17
    shallow_since = "1612701113 +0000",
)

#############################################
git_repository(
    name = "com_google_googletest",
    remote = "git://github.com/google/googletest.git",
    commit = "609281088cfefc76f9d0ce82e1ff6c30cc3591e5",  # current as of 2021/02/17
    shallow_since = "1613065794 -0500",
)

#############################################
git_repository(
    name = "com_google_absl",
    commit = "143a27800eb35f4568b9be51647726281916aac9",  # current as of 2021/02/17
    shallow_since = "1613186346 -0500",
    remote = "git://github.com/abseil/abseil-cpp.git",
)

#############################################
new_git_repository(
    name = "eigen",
    build_file = "@//:BUILD.eigen",
    remote = "https://gitlab.com/libeigen/eigen.git",
    commit = "9b51dc7972c9f64727e9c8e8db0c60aaf9aae532",  # current as of 2021/02/17
    shallow_since = "1613584163 +0000",
)

#############################################
git_repository(
    name = "bazel_rules",
    commit = "7bedda9b65feaa1efab8d9cd77c4c1a8b667b042",  # current as of 2021/02/17
    shallow_since = "1606611670 -0800",
    remote = "git://github.com/bcsgh/bazel_rules.git",
)
