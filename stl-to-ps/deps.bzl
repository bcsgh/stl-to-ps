load("@bazel_rules//repositories:repositories.bzl", "load_absl", "load_skylib")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def get_deps():
    #############################################
    # Bazel Skylib.
    load_skylib()

    #############################################
    git_repository(
        name = "com_googlesource_code_re2",
        commit = "974f44c8d45242e710dc0a85a4defffdb3ce07fc",  # current as of 2023/11/12
        remote = "https://github.com/google/re2.git",
        shallow_since = "1699394483 +0000",
    )

    #############################################
    load_absl()

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
