workspace(name = "stl_to_ps")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

# Note:
# local_repository(name = "...", path = "/home/...")

#############################################
# needed by com_github_glog_glog
git_repository(
    name = "com_github_gflags_gflags",
    commit = "827c769e5fc98e0f2a34c47cef953cc6328abced",  # current as of 2021/02/17
    remote = "git://github.com/gflags/gflags.git",
    shallow_since = "1604052972 +0000",
)

#############################################
git_repository(
    name = "com_github_glog_glog",
    commit = "764162569a26da4401a8b59c96ca3525d7618a28",  # current as of 2021/06/15
    remote = "git://github.com/google/glog.git",
    shallow_since = "1623780547 +0200",
)

#############################################
git_repository(
    name = "com_googlesource_code_re2",
    commit = "4244cd1cb492fa1d10986ec67f862964c073f844",  # current as of 2021/06/10
    remote = "git://github.com/google/re2.git",
    shallow_since = "1621596331 +0000",
)

#############################################
git_repository(
    name = "com_google_googletest",
    commit = "e2239ee6043f73722e7aa812a459f54a28552929",  # current as of 2021/06/15
    remote = "git://github.com/google/googletest.git",
    shallow_since = "1623433346 -0700",
)

#############################################
git_repository(
    name = "com_google_absl",
    commit = "311bbd2e50ea35e921a08186840d3b6ca279e880",  # current as of 2021/06/15
    remote = "git://github.com/abseil/abseil-cpp.git",
    shallow_since = "1623359638 -0400",
)

#############################################
git_repository(
    name = "bazel_rules",
    commit = "b13bb0a8ff74dc0ce06470ff07a105e377fc5fdc",  # current as of 2021/06/15
    remote = "git://github.com/bcsgh/bazel_rules.git",
    shallow_since = "1623790590 -0700",
)

load("@bazel_rules//repositories:repositories.bzl", "eigen")

#############################################
eigen()
