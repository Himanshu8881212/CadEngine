"""LMCAD sampling bridge — drive the Rust hybrid CAD kernel as the geometry
engine for the solvers in this package.

LMCAD is the geometry authority: this module is the voxel coupling point onto
the LMCAD ``kernel-api`` JSON program surface, producing the two-array voxel
contract:

- ``sample_part(program, origin_mm, voxel_size_mm, shape, out_npy)`` runs an
  LMCAD JSON program whose final bound solid (or implicit tree) is sampled
  into ``solid_fraction.npy`` in exactly the ``agents/_schema.md §4``
  encoding (float32, C-order, ``rho[i,j,k]`` with ``i↔x``, voxel centers at
  ``origin + (idx+0.5)·h``). Verified against ``validate_solid_fraction_array``
  and consumed unmodified by ``physics.reference_fea``.
- ``region_kind_from_regions(regions, shape, voxel_size_mm, origin_mm)``
  builds ``region_kind.npy`` from ``physics.regions()`` selectors using the
  SAME resolver the FEA uses (``physics.selectors.resolve_selector``)
  — this removes the region-sampling logic that ``Program.cs`` duplicated
  in C#.
- ``sample_part_file(geometry_json_path, physics_module, out_dir)`` is the
  Designer-agent plumbing on top of the two calls above: it reads the grid
  contract (``voxel_grid_shape``/``voxel_size_mm``/``voxel_origin_mm``) and
  ``regions()`` from the part's ``physics.py`` and materialises BOTH
  ``initial/*.npy`` arrays from a ``spec/geometry.json`` LMCAD program
  (``{"solid": "<id>", "ops": [...]}``), cross-encoding enforced.
- ``emit_stl_gated(rho_npy, voxel_size_mm, origin_mm, out_stl)`` meshes an
  optimized density through LMCAD's redistance + narrow-band pipeline: the
  mesh is WATERTIGHT or the call fails loudly (the raw-marching-cubes
  ``render.emit_stl`` can be swapped for this per part). Returns the
  ``emit_stl`` contract dict: ``{ok, volume_mm3, num_triangles, watertight}``.

The LMCAD side of the contract lives in ``kernel-api`` (`API.md`, "ACE /
voxel-physics bridge"; ops ``sample_density_grid`` / ``mesh_density_grid``;
round-trip pinned by ``crates/kernel-api/tests/bridge.rs``).

LMCAD kernel expected at ``LMCAD_KERNEL_API`` (env var) or the default
release build path below. B-rep booleans/fillets/holes/parts-catalog,
STEP export, assemblies and FDM print gates all become available to the
Designer through the same JSON surface — see LMCAD's API.md.
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import subprocess
import tempfile
from pathlib import Path
from types import ModuleType

import numpy as np

#: tools/analyzers/physics/sampling.py -> repo root; the release build the
#: analyzers ship against. (In ACE this was an absolute path into the
#: maintainer's LMCAD checkout — in-tree it is simply relative to this file.)
_DEFAULT_KERNEL = str(
    Path(__file__).resolve().parents[3] / "target" / "release" / "kernel-api"
)


def kernel_path() -> str:
    """Path to the kernel-api binary (env ``LMCAD_KERNEL_API`` overrides)."""
    p = os.environ.get("LMCAD_KERNEL_API", _DEFAULT_KERNEL)
    if not Path(p).exists():
        raise FileNotFoundError(
            f"LMCAD kernel-api binary not found at {p!r} — build it with "
            "`cargo build -p kernel-api --release` or set LMCAD_KERNEL_API."
        )
    return p


def run_program(program: dict, out_dir: str | Path) -> dict:
    """Run an LMCAD JSON program; return the parsed report (raises on failure)."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", suffix=".json", dir=out_dir, delete=False
    ) as f:
        json.dump(program, f)
        prog_path = f.name
    proc = subprocess.run(
        [kernel_path(), "run", prog_path, "--out-dir", str(out_dir)],
        capture_output=True,
        text=True,
        timeout=600,
    )
    report = json.loads(proc.stdout) if proc.stdout.strip() else {}
    if proc.returncode != 0:
        raise RuntimeError(
            f"LMCAD program failed (exit {proc.returncode}):\n"
            f"{json.dumps(report, indent=2)[:4000]}\n{proc.stderr[:2000]}"
        )
    return report


def sample_part(
    geometry_ops: list[dict],
    solid_id: str,
    origin_mm: tuple[float, float, float],
    voxel_size_mm: float,
    shape: tuple[int, int, int],
    out_npy: str | Path,
    supersample: int = 2,
) -> np.ndarray:
    """Build geometry via LMCAD ops and sample it to ``solid_fraction.npy``.

    ``geometry_ops`` is the op list of an LMCAD JSON program; ``solid_id``
    names the bound solid to sample. The array is written to ``out_npy``
    AND returned (validated float32, C-order).
    """
    out_npy = Path(out_npy)
    program = {
        "ops": geometry_ops
        + [
            {
                "id": "_ace_grid",
                "op": "sample_density_grid",
                "in": solid_id,
                "origin": list(origin_mm),
                "voxel": voxel_size_mm,
                "shape": list(shape),
                "supersample": supersample,
                "file": out_npy.name,
            }
        ]
    }
    run_program(program, out_npy.parent)
    rho = np.load(out_npy)
    assert rho.dtype == np.float32 and rho.shape == tuple(shape), (
        f"bridge contract violation: {rho.dtype} {rho.shape}"
    )
    return rho


def region_kind_from_regions(
    regions: list[dict],
    shape: tuple[int, int, int],
    voxel_size_mm: float,
    origin_mm: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> np.ndarray:
    """``physics.regions()`` → the string ``region_kind`` array, resolved by
    the SAME selector engine the FEA uses (no more C# duplication)."""
    from .selectors import resolve_selector

    kind = np.full(shape, "design", dtype=object)
    order = {"design": 0, "void": 1, "fixed": 2, "frozen": 3}
    for region in sorted(regions, key=lambda r: order.get(r.get("kind"), 0)):
        mask = resolve_selector(
            region["selector"], shape, float(voxel_size_mm), origin_mm
        )
        kind[mask] = region["kind"]
    return kind


def load_physics_module(physics_path: str | Path) -> ModuleType:
    """Import a part's ``spec/physics.py`` from its file path.

    Each part's physics module is imported under a path-hashed name so two
    parts' ``physics`` modules never collide in ``sys.modules``.
    """
    physics_path = Path(physics_path).resolve()
    if not physics_path.is_file():
        raise FileNotFoundError(f"physics module not found: {physics_path}")
    tag = hashlib.sha1(str(physics_path).encode("utf-8")).hexdigest()[:12]
    spec = importlib.util.spec_from_file_location(
        f"_lmcad_physics_{tag}", physics_path
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot build import spec for {physics_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sample_part_file(
    geometry_json_path: str | Path,
    physics_module: ModuleType | str | Path,
    out_dir: str | Path,
    supersample: int = 2,
) -> dict:
    """The Designer plumbing: ``spec/geometry.json`` + ``physics.py`` →
    ``initial/solid_fraction.npy`` + ``initial/region_kind.npy``.

    ``geometry.json`` is ``{"solid": "<id>", "ops": [...]}`` — an LMCAD JSON
    program (op list, see LMCAD's API.md) whose bound solid named by
    ``"solid"`` is the part's initial geometry. ``physics_module`` is the
    part's imported ``physics`` module (or a path to ``physics.py``); its
    ``voxel_grid_shape()`` / ``voxel_size_mm()`` / ``voxel_origin_mm()`` fix
    the grid and ``regions()`` drives ``region_kind`` through
    ``region_kind_from_regions`` (the FEA's own selector engine).

    The two arrays are written into ``out_dir`` in the ``agents/_schema.md``
    §4 encoding, with the cross-encoding enforced belt-and-braces after
    sampling: frozen/fixed voxels → 1.0, void voxels → 0.0 (the geometry
    should already CONTAIN the frozen volumes — the clamp is a guarantee,
    not a substitute for building them). Design voxels keep the geometry's
    supersampled solid fraction: the drawn solid IS the initial density.

    Returns a summary dict (paths, shape, mean fraction, region counts).
    Raises ``RuntimeError`` carrying the kernel report — which names the
    failing op id and reason — if the LMCAD program fails; raises
    ``ValueError`` on a malformed ``geometry.json``.
    """
    geometry_json_path = Path(geometry_json_path)
    doc = json.loads(geometry_json_path.read_text(encoding="utf-8"))
    if not isinstance(doc, dict) or not isinstance(doc.get("ops"), list):
        raise ValueError(
            f"{geometry_json_path}: geometry.json must be an object with an "
            '"ops" array of LMCAD ops'
        )
    solid_id = doc.get("solid")
    if not isinstance(solid_id, str) or not solid_id:
        raise ValueError(
            f'{geometry_json_path}: missing required top-level "solid" key — '
            "it must name the id of the op whose bound solid is the part"
        )

    if isinstance(physics_module, (str, Path)):
        physics_module = load_physics_module(physics_module)

    shape = tuple(int(n) for n in physics_module.voxel_grid_shape())
    voxel_size_mm = float(physics_module.voxel_size_mm())
    if hasattr(physics_module, "voxel_origin_mm"):
        origin_mm = tuple(float(v) for v in physics_module.voxel_origin_mm())
    else:
        origin_mm = (0.0, 0.0, 0.0)
    regions = physics_module.regions()

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    sf_path = out_dir / "solid_fraction.npy"
    rk_path = out_dir / "region_kind.npy"

    rho = sample_part(
        doc["ops"], solid_id, origin_mm, voxel_size_mm, shape, sf_path,
        supersample=supersample,
    )
    kind = region_kind_from_regions(regions, shape, voxel_size_mm, origin_mm)

    # Belt-and-braces cross-encoding (schema §4): void → 0.0, frozen/fixed
    # → 1.0. Sampling puts fractional values on frozen-region boundaries;
    # the contract pins them.
    rho = np.clip(rho, 0.0, 1.0)
    rho[kind == "void"] = 0.0
    rho[(kind == "frozen") | (kind == "fixed")] = 1.0
    rho = rho.astype(np.float32)
    np.save(sf_path, rho)
    np.save(rk_path, kind, allow_pickle=True)

    return {
        "ok": True,
        "solid_fraction_path": str(sf_path),
        "region_kind_path": str(rk_path),
        "voxel_grid_shape": list(shape),
        "voxel_size_mm": voxel_size_mm,
        "voxel_origin_mm": list(origin_mm),
        "solid_fraction_mean": float(rho.mean()),
        "region_counts": {
            k: int((kind == k).sum())
            for k in ("frozen", "fixed", "design", "void")
        },
    }


def emit_stl_gated(
    rho: np.ndarray,
    voxel_size_mm: float,
    origin_mm: tuple[float, float, float],
    out_stl: str | Path,
    iso: float = 0.5,
) -> dict:
    """Mesh an optimized density field through LMCAD's gated pipeline.

    Drop-in for the ``render.emit_stl`` contract: returns ``{ok, volume_mm3,
    num_triangles, watertight, issues}`` — but ``watertight`` is enforced by
    the kernel (redistance → narrow-band dual contouring → manifold heal),
    not merely reported.
    """
    out_stl = Path(out_stl)
    out_dir = out_stl.parent
    npy_path = out_dir / "_lmcad_rho.npy"
    np.save(npy_path, np.asarray(rho, dtype=np.float32))
    program = {
        "ops": [
            {
                "id": "_ace_stl",
                "op": "mesh_density_grid",
                "npy": npy_path.name,
                "origin": list(origin_mm),
                "voxel": voxel_size_mm,
                "iso": iso,
                "file": out_stl.name,
            }
        ]
    }
    try:
        report = run_program(program, out_dir)
    except RuntimeError as exc:
        return {
            "ok": False,
            "volume_mm3": 0.0,
            "num_triangles": 0,
            "watertight": False,
            "issues": [str(exc)],
        }
    finally:
        npy_path.unlink(missing_ok=True)
    measures = {}
    for op in report.get("ops", []):
        if op.get("id") == "_ace_stl":
            measures = op.get("measures", {})
    return {
        "ok": bool(measures.get("watertight", False)),
        "volume_mm3": float(measures.get("volume_mm3", 0.0)),
        "num_triangles": int(measures.get("num_triangles", 0)),
        "watertight": bool(measures.get("watertight", False)),
        "issues": [],
    }
