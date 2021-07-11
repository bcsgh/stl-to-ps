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
    commit = "42ce901f286c6140329ce12a7cba379637e361db",  # current as of 2021/07/10
    remote = "git://github.com/google/glog.git",
    shallow_since = "1625509992 +0200",
)

#############################################
git_repository(
    name = "com_googlesource_code_re2",
    commit = "892ed217d3ac114d51fa0738f82783282a5e8230",  # current as of 2021/07/10
    remote = "git://github.com/google/re2.git",
    shallow_since = "1625789526 +0000",
)

#############################################
git_repository(
    name = "com_google_googletest",
    commit = "8d51ffdfab10b3fba636ae69bc03da4b54f8c235",  # current as of 2021/07/10
    remote = "git://github.com/google/googletest.git",
    shallow_since = "1625837293 -0400",
)

#############################################
git_repository(
    name = "com_google_absl",
    commit = "b06e719ee985ecd63e0dffbc68499549216f817f",  # current as of 2021/07/10
    remote = "git://github.com/abseil/abseil-cpp.git",
    shallow_since = "1625837242 -0400",
)

#############################################
git_repository(
    name = "bazel_rules",
    commit = "983056fbf48c51d0342401522ad07155dc07beb0",  # current as of 2021/07/10
    remote = "git://github.com/bcsgh/bazel_rules.git",
    shallow_since = "1625971439 -0700",
)

load("@bazel_rules//repositories:repositories.bzl", "eigen")

#############################################
eigen()
