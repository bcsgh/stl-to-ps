workspace(name = "stl_to_ps")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")


# Note:
# local_repository(name = "...", path = "/home/...")

#############################################
git_repository(
    name = "bazel_skylib",
    commit = "9c9beee7411744869300f67a98d42f5081e62ab3",  # current as of 2023/11/12
    remote = "https://github.com/bazelbuild/bazel-skylib.git",
    shallow_since = "1699201005 -0500",
)

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")

bazel_skylib_workspace()

#############################################
git_repository(
    name = "com_google_googletest",
    commit = "b10fad38c4026a29ea6561ab15fc4818170d1c10",  # current as of 2023/11/12
    remote = "https://github.com/google/googletest.git",
    shallow_since = "1698701593 -0700",
)

#############################################
git_repository(
    name = "bazel_rules",
    commit = "be9e3fa50c41cf9a1e93d2288ce02c67047d71c3",  # current as of 2023/11/16
    remote = "https://github.com/bcsgh/bazel_rules.git",
    shallow_since = "1700184387 -0800",
)

load("@bazel_rules//repositories:repositories.bzl", "eigen", "jsoncpp")

#############################################
register_toolchains("@bazel_rules//parser:linux_flex_bison")

#############################################
eigen()

jsoncpp()

load("@stl_to_ps//stl-to-ps:deps.bzl", stl_to_ps_deps = "get_deps")
stl_to_ps_deps()
