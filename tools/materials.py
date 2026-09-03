#!/usr/bin/env python3
# LMCAD forwarding shim — materials.py moved to tools/analyzers/materials.py on 2026-09-02 (the
# tools/ re-organisation; the map is tools/_layout.py). Run as a script it
# executes the real file with the same argv, stdout and exit code, so
# `python3 tools/materials.py job.json` keeps working for every campaign run_all.sh,
# CI job and the MCP server; imported as a module it hands back the real module.
# Edit the real file, never this one.
import os
import sys

_target = os.path.join(os.path.dirname(os.path.abspath(__file__)), "analyzers", "materials.py")
if __name__ == "__main__":
    import runpy

    sys.argv[0] = _target
    runpy.run_path(_target, run_name="__main__")
else:
    import importlib.util

    _spec = importlib.util.spec_from_file_location(__name__, _target)
    _mod = importlib.util.module_from_spec(_spec)
    sys.modules[__name__] = _mod
    _spec.loader.exec_module(_mod)
