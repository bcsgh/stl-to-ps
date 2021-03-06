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

def stl2pdf(name = None, script = None, deps = []):
    """Process .stl files into .pdf files.

    Args:
      name: The target name.
      script: The file describing the page layouts.
      deps: The list of .stl files used by `script`.
    """
    if not script:
        fail("script must be provided")

    root = name
    cmd = "$(location @stl_to_ps//stl-to-ps:stl-to-ps) --output=$@"
    cmd += " --load_path=$(BINDIR),$(GENDIR)"
    cmd += " --script=$(location %s)" % script
    cmd += " --name=%s" % root
    srcs = [script] + deps

    ps = root + ".ps"
    pdf = root + ".pdf"
    native.genrule(
        name = name + "_ps",
        srcs = srcs,
        outs = [ps],
        tools = ["@stl_to_ps//stl-to-ps:stl-to-ps"],
        cmd = cmd,
    )

    native.genrule(
        name = name,
        srcs = [ps],
        outs = [pdf],
        cmd = "ps2pdf $< $@",
    )

def scad_binary(name = None, src = None, deps = []):
    """Process .scad (OpenSCAD) files into .stl files.

    Args:
      name: The target name.
      src: The top level SCAD file.
      deps: SCAD files that are used by src.
    """
    native.genrule(
        name = name,
        srcs = [src] + deps,
        outs = [src[:src.find(".")] + ".stl"],
        cmd = ("openscad -o $@ $(location :%s)" % src),
    )
