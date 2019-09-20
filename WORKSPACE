workspace(name = "stl_to_ps")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")

#############################################
git_repository(
    name = "com_github_gflags_gflags",
    tag = "v2.2.2",  # current as of 2019/9/19
    remote = "git://github.com/gflags/gflags.git",
)

#############################################
git_repository(
    name = "com_googlesource_code_re2",
    tag = "2019-09-01",  # current as of 2019/9/19
    remote = "git://github.com/google/re2.git",
)

#############################################
git_repository(
    name = "com_google_googletest",
    tag = "release-1.8.1",  # current as of 2019/9/19
    remote = "git://github.com/google/googletest.git",
)

#############################################
git_repository(
    name = "com_google_absl",
    commit = "ddf8e52a2918dd0ccec75d3e2426125fa3926724",  # current as of 2019/9/19
    remote = "git://github.com/abseil/abseil-cpp.git",
)

#############################################
new_git_repository(
    name = "eigen",
    tag = "3.3.7",  # current as of 2019/9/19
    remote = "git://github.com/eigenteam/eigen-git-mirror.git",
    build_file = "@//:BUILD.eigen",
)
