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
    commit = "c1499f64a8fb2eae2de79ed7195cf248e706efc9",  # current as of 2021/06/10
    remote = "git://github.com/google/glog.git",
    shallow_since = "1623050920 +0200",
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
    commit = "aa533abfd4232b01f9e57041d70114d5a77e6de0",  # current as of 2021/06/10
    remote = "git://github.com/google/googletest.git",
    shallow_since = "1623242719 -0400",
)

#############################################
git_repository(
    name = "com_google_absl",
    commit = "8f92175783c9685045c50f227e7c10f1cddb4d58",  # current as of 2021/06/10
    remote = "git://github.com/abseil/abseil-cpp.git",
    shallow_since = "1623281200 -0400",
)

#############################################
git_repository(
    name = "bazel_rules",
    commit = "91f468616384061a36754a4bf8ec2d8e68f222b9",  # current as of 2021/06/10
    remote = "git://github.com/bcsgh/bazel_rules.git",
    shallow_since = "1623350173 -0700",
)

load("@bazel_rules//repositories:repositories.bzl", "eigen")

#############################################
eigen()
