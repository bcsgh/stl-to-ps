workspace(name = "stl_to_ps")

#############################################
git_repository(
    name = "com_github_gflags_gflags",
    tag = "v2.2.1",  # current as of 2018/6/30
    remote = "git://github.com/gflags/gflags.git",
)

#############################################
git_repository(
    name = "com_googlesource_code_re2",
    tag = "2018-07-01",  # current as of 2018/7/2
    remote = "git://github.com/google/re2.git",
)

#############################################
git_repository(
    name = "com_google_googletest",
    # tag = "release-1.9.0",  # not yet ready as of 2018/6/30
    commit = "1865ecaf1779c2a2f210ca3768aa030206ef74ba",  # needed to get BUILD.bazel
    remote = "git://github.com/google/googletest.git",
)

#############################################
git_repository(
    name = "com_google_absl",
    commit = "ba8d6cf07766263723e86736f20a51c1c9c67b19",  # current as of 2018/6/30
    remote = "git://github.com/abseil/abseil-cpp.git",
)

#############################################
new_git_repository(
    name = "eigen",
    tag = "3.3.4",  # current as of 2018/6/30
    remote = "git://github.com/eigenteam/eigen-git-mirror.git",
    build_file = "BUILD.eigen",
)
