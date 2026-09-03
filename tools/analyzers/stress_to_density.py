#!/usr/bin/env python3
"""stress_to_density.py — von Mises stress .npy -> graded-density .npy in [floor, ceil].

The simulation->geometry hinge of the FEA-driven grading loop:
``tools/ace_fea_runner.py`` writes ``stress_field.npy``; this tool maps it to
a density grid; ``kernel-implicit``'s ``GridField``
(``crates/kernel-implicit/src/grid_field.rs``) loads that density as a grade
law for ``Node::offset_by`` — thicker lattice walls where the part works
hardest. Any (nx, ny, nz) float scalar field works; "stress" is just the
canonical customer.

Usage: stress_to_density.py <stress.npy> [--floor 0.15] [--ceil 1.0]
                            [--gamma 1.0] [--clip-percentile 99]

Mapping: vm_clip = percentile(vm[vm > 0], clip_percentile) — positive voxels
only, so the void sea of exact zeros cannot drag the clip point down (falls
back to the whole grid when nothing is positive); then
t = clip(vm / vm_clip, 0, 1) and density = floor + (ceil - floor) * t**gamma.
gamma > 1 concentrates material at hot spots, gamma < 1 spreads it. An
all-zero field maps everywhere to floor (vm_clip == 0 guard, noted in the
receipt via stress_max_clipped = 0). Non-finite voxels are refused, not
laundered — a NaN would poison the kernel-side trilinear sampling.

Output: <stem>_density.npy (float32, C-order, same shape) next to the input.
Receipt (LAST stdout line, one JSON object — voxelize_stl.py convention):
{ok, in, out, shape, stress_min, stress_max_clipped, floor, ceil, gamma,
clip_percentile, density_mean}. On failure: {ok: false, error} and exit 1.
"""
import argparse
import json
import os
import sys

import numpy as np


def main():
	ap = argparse.ArgumentParser(description="Map a von Mises stress .npy to a [floor, ceil] density .npy for GridField grading.")
	ap.add_argument("stress_npy", help="input (nx, ny, nz) float .npy, e.g. an ace_fea stress_field.npy")
	ap.add_argument("--floor", type=float, default=0.15, help="density at zero stress (default 0.15 — keep a printable minimum wall)")
	ap.add_argument("--ceil", type=float, default=1.0, help="density at/above the clip stress (default 1.0)")
	ap.add_argument("--gamma", type=float, default=1.0, help="response exponent on the normalized stress (default 1.0 = linear)")
	ap.add_argument("--clip-percentile", type=float, default=99.0, help="percentile of positive stress mapped to density ceil; outliers above it saturate (default 99)")
	args = ap.parse_args()

	if not (np.isfinite(args.floor) and np.isfinite(args.ceil) and args.floor <= args.ceil):
		raise ValueError(f"need finite floor <= ceil, got floor={args.floor} ceil={args.ceil}")
	if not (np.isfinite(args.gamma) and args.gamma > 0.0):
		raise ValueError(f"need gamma > 0, got {args.gamma}")
	if not (0.0 < args.clip_percentile <= 100.0):
		raise ValueError(f"need 0 < clip-percentile <= 100, got {args.clip_percentile}")

	vm = np.load(args.stress_npy, allow_pickle=False)
	if vm.ndim != 3:
		raise ValueError(f"expected a 3-D (nx, ny, nz) grid, got shape {vm.shape}")
	vm = np.asarray(vm, dtype=np.float64)
	if not np.isfinite(vm).all():
		raise ValueError(f"{int((~np.isfinite(vm)).sum())} non-finite voxel(s) — clean the field first")

	positive = vm[vm > 0.0]
	vm_clip = float(np.percentile(positive if positive.size else vm, args.clip_percentile))
	if vm_clip > 0.0:
		t = np.clip(vm / vm_clip, 0.0, 1.0)
	else:
		t = np.zeros_like(vm)  # all-zero (or non-positive) field: everything coasts at floor
	density = (args.floor + (args.ceil - args.floor) * t ** args.gamma).astype(np.float32)

	stem, _ = os.path.splitext(args.stress_npy)
	out = stem + "_density.npy"
	np.save(out, np.ascontiguousarray(density))

	print(json.dumps({
		"ok": True,
		"in": args.stress_npy,
		"out": out,
		"shape": list(vm.shape),
		"stress_min": float(vm.min()),
		"stress_max_clipped": vm_clip,
		"floor": args.floor,
		"ceil": args.ceil,
		"gamma": args.gamma,
		"clip_percentile": args.clip_percentile,
		"density_mean": float(density.mean()),
	}))


if __name__ == "__main__":
	try:
		main()
	except Exception as e:  # receipt is the contract: emit ok:false, exit 1
		print(json.dumps({"ok": False, "error": f"{type(e).__name__}: {e}"}))
		sys.exit(1)
