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
    commit = "9dc1107f88d3a1613d61b80040d83c1c1acbac3d",  # current as of 2021/12/17
    remote = "https://github.com/google/glog.git",
    shallow_since = "1639566274 +0100",
)

#############################################
git_repository(
    name = "com_googlesource_code_re2",
    commit = "d826d9fcb68c62996c1b7c0a45d604e22d814952",  # current as of 2021/12/17
    remote = "https://github.com/google/re2.git",
    shallow_since = "1637254443 +0000",
)

#############################################
git_repository(
    name = "com_google_googletest",
    commit = "97a467571a0f615a4d96e79e4399c43221ca1232",  # current as of 2021/12/17
    remote = "https://github.com/google/googletest.git",
    shallow_since = "1639586168 -0800",
)

#############################################
git_repository(
    name = "com_google_absl",
    commit = "52d41a9ec23e39db7e2cbce5c9449506cf2d3a5c",  # current as of 2021/12/17
    remote = "https://github.com/abseil/abseil-cpp.git",
    shallow_since = "1639580175 -0500",
)

#############################################
git_repository(
    name = "bazel_rules",
    commit = "69f0ea11bbea5166cf0e735cda098736c2386aca",  # current as of 2021/12/17
    remote = "https://github.com/bcsgh/bazel_rules.git",
    shallow_since = "1639764084 -0800",
)

load("@bazel_rules//repositories:repositories.bzl", "eigen")

#############################################
eigen()
