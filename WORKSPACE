workspace(name = "stl_to_ps")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")


# Note:
# local_repository(name = "...", path = "/home/...")

#############################################
git_repository(
    name = "bazel_skylib",
    commit = "5bfcb1a684550626ce138fe0fe8f5f702b3764c3",  # current as of 2023/01/02
    remote = "https://github.com/bazelbuild/bazel-skylib.git",
    shallow_since = "1668623372 +0100",
)

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")

bazel_skylib_workspace()

#############################################
git_repository(
    name = "com_google_googletest",
    commit = "3026483ae575e2de942db5e760cf95e973308dd5",  # current as of 2022/10/25
    remote = "https://github.com/google/googletest.git",
    shallow_since = "1666712359 -0700",
)

#############################################
git_repository(
    name = "bazel_rules",
    commit = "3404db100ba321134ef63c6c872ce8dca6db024d",  # current as of 2023/01/29
    remote = "https://github.com/bcsgh/bazel_rules.git",
    shallow_since = "1675037889 -0800"
)

load("@bazel_rules//repositories:repositories.bzl", "eigen", "jsoncpp")

#############################################
register_toolchains("@bazel_rules//parser:linux_flex_bison")

#############################################
eigen()

jsoncpp()

load("@stl_to_ps//stl-to-ps:deps.bzl", stl_to_ps_deps = "get_deps")
stl_to_ps_deps()
