def build_test(name = None, targets = []):
  native.sh_test(
      name = name,
      srcs = ["//tools:blank.sh"],
      data = targets,
      timeout="short",
  )
