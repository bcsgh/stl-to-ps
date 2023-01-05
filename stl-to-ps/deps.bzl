load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def get_deps():
    #############################################
    # Bazel Skylib.
    http_archive(
        name = "bazel_skylib",
        urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/1.2.1/bazel-skylib-1.2.1.tar.gz"],
        sha256 = "f7be3474d42aae265405a592bb7da8e171919d74c16f082a5457840f06054728",
    )

    #############################################
    git_repository(
        name = "com_googlesource_code_re2",
        commit = "7a65faf439295e941baa6640a717d89c1f13e9cd",  # current as of 2022/10/27
        remote = "https://github.com/google/re2.git",
        shallow_since = "1666860568 +0000",
    )

    #############################################
    git_repository(
        name = "com_google_absl",
        commit = "827940038258b35a29279d8c65b4b4ca0a676f8d",  # current as of 2022/10/27
        remote = "https://github.com/abseil/abseil-cpp.git",
        shallow_since = "1666903548 -0700",
    )

    # For "reasons", these have to be in the WORKSPACE.
    # Juct check for them and tell people how to fix thigns.
    NEEDED = {
        "jsoncpp": "com_github_open_source_parsers_jsoncpp",
        "eigen": "eigen",
    }
    missing = [f for f,r in NEEDED.items() if not native.existing_rule(r)]
    if missing:
        print("\n".join([
            "Missing packages needed by @stl-to-ps//. Add the following to WORKSPACE:\n",
            'load("@bazel_rules//repositories:repositories.bzl", "' + '", "'.join(missing) + '")',
        ] + [
            "%s()" % p for p in missing
        ]) + "\n")
