workspace(name = "stl_to_ps")

#############################################
git_repository(
    name = "com_github_gflags_gflags",
    tag = "v2.2.1",  # current as of 2018/9/23
    remote = "git://github.com/gflags/gflags.git",
)

#############################################
git_repository(
    name = "com_googlesource_code_re2",
    tag = "2018-09-01",  # current as of 2018/9/23
    remote = "git://github.com/google/re2.git",
)

#############################################
git_repository(
    name = "com_google_googletest",
    tag = "release-1.8.1",  # current as of 2018/9/23
    remote = "git://github.com/google/googletest.git",
)

#############################################
git_repository(
    name = "com_google_absl",
    commit = "8ff1374008259719b54a8cb128ef951c02da164c",  # current as of 2018/9/23
    remote = "git://github.com/abseil/abseil-cpp.git",
)

#############################################
new_git_repository(
    name = "eigen",
    tag = "3.3.5",  # current as of 2018/9/23
    remote = "git://github.com/eigenteam/eigen-git-mirror.git",
    build_file = "BUILD.eigen",
)
