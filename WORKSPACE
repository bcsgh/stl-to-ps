workspace(name = "stl_to_ps")

#############################################
git_repository(
    name = "com_github_gflags_gflags",
    tag = "v2.2.1",  # current as of 2018/4/13
    remote = "git://github.com/gflags/gflags.git",
)

#############################################
git_repository(
    name = "com_googlesource_code_re2",
    tag = "2018-04-01",  # current as of 2018/4/13
    remote = "git://github.com/google/re2.git",
)

#############################################
git_repository(
    name = "com_google_googletest",
    # tag = "release-1.9.0",  # not yet ready as of 2018/4/13
    commit = "1865ecaf1779c2a2f210ca3768aa030206ef74ba",  # needed to get BUILD.bazel
    remote = "git://github.com/google/googletest.git",
)

#############################################
git_repository(
    name = "com_google_absl",
    commit = "a7e522daf1ec9cda69b356472f662142dd0c1215",  # current as of 2018/4/13
    remote = "git://github.com/abseil/abseil-cpp.git",
)

#############################################
# From com_google_absl/WORKSPACE
git_repository(
    name = "com_googlesource_code_cctz",
    tag = "v2.2",  # current as of 2018/4/13
    remote = "git://github.com/google/cctz.git",
)

#############################################
new_git_repository(
    name = "eigen",
    tag = "3.3.4",  # current as of 2018/4/13
    remote = "git://github.com/eigenteam/eigen-git-mirror.git",
    build_file = "BUILD.eigen",
)
