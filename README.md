<!-- Generated with Stardoc: http://skydoc.bazel.build -->

# Bazel rule for `.scad` and `.stl` files.

Bazle/skylark rules to process `.scad` (a text based CAD file format) into
`.stl` files (a file format used for 3d printing) into `.ps` and `.pdf` files
(a file formate used for 2d printing) as blueprints (a type of document used
for general manufacturing).

## `MODULE.bazel`

```
bazel_dep(
    name = "com_github_bcsgh_stl_to_ps",
    version = ...,
)
```

<a id="scad_binary"></a>

## scad_binary

<pre>
load("@com_github_bcsgh_stl_to_ps//stl-to-ps:rule.bzl", "scad_binary")

scad_binary(<a href="#scad_binary-name">name</a>, <a href="#scad_binary-deps">deps</a>, <a href="#scad_binary-src">src</a>, <a href="#scad_binary-out">out</a>)
</pre>

Process .scad (OpenSCAD) files into .stl files.

**ATTRIBUTES**


| Name  | Description | Type | Mandatory | Default |
| :------------- | :------------- | :------------- | :------------- | :------------- |
| <a id="scad_binary-name"></a>name |  A unique name for this target.   | <a href="https://bazel.build/concepts/labels#target-names">Name</a> | required |  |
| <a id="scad_binary-deps"></a>deps |  SCAD files that are used by src.   | <a href="https://bazel.build/concepts/labels">List of labels</a> | optional |  `[]`  |
| <a id="scad_binary-src"></a>src |  The top level SCAD file.   | <a href="https://bazel.build/concepts/labels">Label</a> | required |  |
| <a id="scad_binary-out"></a>out |  The target file name.   | <a href="https://bazel.build/concepts/labels">Label</a> | required |  |


<a id="stl2pdf"></a>

## stl2pdf

<pre>
load("@com_github_bcsgh_stl_to_ps//stl-to-ps:rule.bzl", "stl2pdf")

stl2pdf(<a href="#stl2pdf-name">name</a>, <a href="#stl2pdf-deps">deps</a>, <a href="#stl2pdf-out">out</a>, <a href="#stl2pdf-script">script</a>)
</pre>

Process .stl files into .pdf files.

**ATTRIBUTES**


| Name  | Description | Type | Mandatory | Default |
| :------------- | :------------- | :------------- | :------------- | :------------- |
| <a id="stl2pdf-name"></a>name |  A unique name for this target.   | <a href="https://bazel.build/concepts/labels#target-names">Name</a> | required |  |
| <a id="stl2pdf-deps"></a>deps |  The list of .stl files used by `script`.   | <a href="https://bazel.build/concepts/labels">List of labels</a> | optional |  `[]`  |
| <a id="stl2pdf-out"></a>out |  The target file name.   | <a href="https://bazel.build/concepts/labels">Label</a> | required |  |
| <a id="stl2pdf-script"></a>script |  The file describing the page layouts.   | <a href="https://bazel.build/concepts/labels">Label</a> | required |  |


## Setup (for development)
To configure the git hooks, run `./.git_hooks/setup.sh`
