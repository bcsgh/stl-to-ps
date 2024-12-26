# Copyright (c) 2017, Benjamin Shropshire,
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Bazle/skylark rules to process .scad and .stl files into .pdf files."""

def _stl2pdf_impl(ctx):
    out_ps = ctx.actions.declare_file(ctx.label.name + ".ps")

    inputs = ctx.files.deps + [ctx.file.script]
    load_path = dict([(x.root.path, 0) for x in inputs if x.root.path]).keys()

    args_ps = ctx.actions.args()
    args_ps.add("--output=%s" % out_ps.path)
    args_ps.add("--load_path=%s" % ",".join(load_path))
    args_ps.add("--script=%s" % ctx.file.script.path)
    args_ps.add("--name=%s" % ctx.label.name)

    ctx.actions.run(
        inputs=inputs,
        outputs=[out_ps],
        executable=ctx.file._stl_to_ps,
        arguments = [args_ps]
    )

    out_pdf = ctx.actions.declare_file(ctx.outputs.out.basename)

    args_pdf = ctx.actions.args()
    args_pdf.add(out_ps.path)
    args_pdf.add(out_pdf.path)

    ctx.actions.run(
        inputs=[out_ps],
        outputs=[out_pdf],
        executable="ps2pdf",
        arguments = [args_pdf]
    )

    return [DefaultInfo(
        runfiles=ctx.runfiles(files = ctx.files.deps + [
            ctx.file.script,
            ctx.file._stl_to_ps,
        ]),
    )]

stl2pdf = rule(
    doc = "Process .stl files into .pdf files.",

    implementation = _stl2pdf_impl,
    attrs = {
        "script": attr.label(
            doc="The file describing the page layouts.",
            allow_single_file=True,
            mandatory=True,
        ),
        "deps": attr.label_list(
            doc="The list of .stl files used by `script`.",
            allow_files=True,
        ),
        "out": attr.output(
            doc="The target file name.",
            mandatory=True,
        ),
        "_stl_to_ps": attr.label(
            doc="The stl_to_ps tool.",
            default=Label("//stl-to-ps:stl-to-ps"),
            allow_single_file=True,
        ),
    },
)

def _scad_binary_impl(ctx):
    out = ctx.actions.declare_file(ctx.outputs.out.basename)

    args = ctx.actions.args()
    args.add("-o%s" % out.path)
    args.add(ctx.file.src)

    ctx.actions.run(
        inputs=ctx.files.src + ctx.files.deps,
        outputs=[out],
        executable="openscad",
        arguments = [args]
    )

    return [DefaultInfo(
        runfiles=ctx.runfiles(files = ctx.files.src),
    )]

scad_binary = rule(
    doc = "Process .scad (OpenSCAD) files into .stl files.",

    implementation = _scad_binary_impl,
    attrs = {
        "src": attr.label(
            doc = "The top level SCAD file.",
            allow_single_file=True,
            mandatory=True,
        ),
        "deps": attr.label_list(
            doc = "SCAD files that are used by src.",
            allow_files=True,
        ),
        "out": attr.output(
            doc="The target file name.",
            mandatory=True,
        ),
    },
)
