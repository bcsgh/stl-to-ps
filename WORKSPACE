workspace(name = "stl_to_ps")

#############################################
git_repository(
    name = "com_github_gflags_gflags",
    tag = "v2.2.1",  # current as of 2017/12/19
    remote = "git://github.com/gflags/gflags.git",
)

#############################################
git_repository(
    name = "com_googlesource_code_re2",
    tag = "2017-12-01",  # current as of 2017/12/19
    remote = "git://github.com/google/re2.git",
)

#############################################
git_repository(
    name = "com_google_googletest",
    # tag = "release-1.9.0",  # not yet ready as of 2017/12/19
    commit = "1865ecaf1779c2a2f210ca3768aa030206ef74ba",  # needed to get BUILD.bazel
    remote = "git://github.com/google/googletest.git",
)

#############################################
git_repository(
    name = "com_google_absl",
    commit = "4972c72c5cf2f27e2a0846ce9ff5d377d3f2b7af",  # current as of 2017/12/19
    remote = "git://github.com/abseil/abseil-cpp.git",
)

#############################################
# From com_google_absl/WORKSPACE
git_repository(
    name = "com_googlesource_code_cctz",
    tag = "v2.1",  # current as of 2017/12/19
    remote = "git://github.com/google/cctz.git",
)

#############################################
new_http_archive(
    name = "eigen",
    strip_prefix = "eigen-eigen-5a0156e40feb",
    urls = [
      "https://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz",  # current as of 2017/12/19

      # NOTE: for some reason bazel doesn't like downloading the above file.
      # As a work around, download it localy and use a file:// URL
      #"file:///.../eigen-3.3.4.tar.gz",
    ],
    sha256 = "4286e8f1fabbd74f7ec6ef8ef31c9dbc6102b9248a8f8327b04f5b68da5b05e1",
    build_file = "BUILD.eigen",
)
