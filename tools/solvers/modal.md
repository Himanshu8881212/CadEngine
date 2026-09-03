# modal — hex8 natural frequencies + mode shapes (voxel FEA)

- **Runner**: `tools/analyzers/ace_modal_runner.py` — K/M assembly, occupancy, and fixture/selector handling IMPORTED from ACE `engine.verify.fea` (the exact matrices `reference_fea`/`reference_modal` build); only the eigensolve layer is local, because ACE returns no eigenvectors and refuses free-free. · **Gates**: `tools/tests/test_ace_modal_buckling.py` · legacy pin `tools/validation/ace_modal_validation.py` (ACE `reference_modal`) · manifest `tools/manifests/ace_modal.manifest.json`
- **Physics**: undamped linear free vibration about the unloaded state — no damping, no preload / stress stiffening, no plasticity. Isotropic homogeneous material; density must be > 0.
- **Governing equations**: `K phi = omega^2 M phi`, `f = omega / 2 pi`. Participation `Gamma_d = phi^T M r_d` (mass-normalised phi), effective modal mass `Gamma_d^2`.
- **Discretization**: trilinear hex8 on the voxel grid (binary occupancy rho >= 0.5); **lumped (row-sum) diagonal mass** — positive-definite by construction, makes `A = M^-1/2 K M^-1/2` an exact standard-form reduction (eigenvectors come out mass-normalised for free), low-mode accuracy same order as the hex8 stiffness error (both push f HIGH, converging down). Eigensolve: `scipy.sparse.linalg.eigsh` shift-invert `sigma=0` (fixed, SPD) / `sigma = -(2 pi 1e-3 f_long)^2` with `f_long = sqrt(E/rho)/(2 d_bbox)` (free-free — makes the singular K factorisable; method stated in the receipt); dense `eigh` under 400 DOF; eigenvalues <= 1e-6 x max classified rigid-body.

## I/O contract (JSON job -> .npy mode shapes + JSON receipt on last stdout line)
```
{out_dir, voxel_mm, origin_mm?,
 ops+solid+shape[+supersample] | npy | stl+shape,    # STL parity-filled by invoking tools/analyzers/voxelize_stl.py (one bridge, not reimplemented)
 material: "PLA" | {youngs_modulus_pa, poisson, density_kg_m3},   # string key resolves via tools/analyzers/materials.py; density_kg_m3 > 0 enforced
 fixtures: [{kind: clamped|pinned|slider, region_selector (bbox|plane|cylinder|sphere|all), dof_constrained?}],
 free_free?: true,                                   # EXPLICIT opt-in; no fixtures without it = refusal, never a silent fallback
 n_modes?: 6, regions?}
```
Outputs: `mode_shape_NN.npy` per elastic mode — (nx,ny,nz) float32 C-order per-VOXEL modal displacement magnitude (unit-max, zeros in void; same layout as ace_fea `disp_field.npy`, loadable by `GridField::from_npy_file`; per-element field, so GridField origin = job origin + voxel/2). Receipt: `frequencies_hz` (elastic, ascending), `rigid_body_modes_hz`, `boundary`, per-mode `participation` (effective_mass_kg/fraction + kinetic_fraction per axis — how the gates tell bending from torsion), `total_active_mass_kg`, `eigensolve` method, fixture node-count receipts, provenance envelope. Failure = `{ok:false, error}` + **exit 0** (the legacy bridge convention — the JSON line is the contract, not the exit code).

## Benchmark gates (measured 2026-07-30, all green; bands frozen from these numbers)
| gate | closed form | measured | band asserted |
|---|---|---|---|
| cantilever modes 1-3 (60x6x3 mm, L/h=20) | Euler-Bernoulli `f_i = (b_i L)^2/2pi L_eff^2 sqrt(EI/rho A)`, `L_eff = L - voxel` (a plane clamp fixes the whole first element layer) | vox 0.75: +2.58/+1.31/-0.56% · vox 0.5: +1.42/+0.19/-1.64% | per-mode ~+-1.3% margin; modes 1-2 must CONVERGE toward EB |
| mode-3 sign flip | Timoshenko shear (absent from EB) lowers the true answer ~2.8% at L/h=20 | fine mode 3 lands -1.64% (below EB, above the Timoshenko limit) | fine band (-3.0%, -0.2%) — NEGATIVE by physics, not by accident |
| free-free beam | 6 rigid modes then `b_i L` = 4.7300, 7.8532 | 6 rigid at <= 7.7e-3 Hz (< 1e-5 x f_el1); z1 +2.37% -> +0.97%, z2 +0.68% -> -0.64% | exactly 6 rigid; rigid < 1e-3 x f_el1; ~+-1.3% bands; z1 converges |
| participation physics | cantilever EB effective-mass fractions 0.613/0.188/0.065 | 0.603/0.187/0.065; total mass = rho h^3 n exactly | banded per mode; mass conservation <= 1e-12 rel |
| mode-shape physics | mode 1 root-quiet/tip-loud; mode 2 node at 0.774 L | root 0.033 / tip 1.000; interior dip 0.114 | root < 0.1, tip > 0.9; dip < 0.25 |
| cross-pin vs ACE | same case, `reference_modal` | max rel diff 3.0e-13 over 8 modes | <= 1e-6 |
| negative controls | no fixtures + no free_free flag; density = 0 | both refuse with pointed errors (in-process + subprocess contract) | must refuse |

## Validity limits / out of scope
- Frequencies are a-few-percent HIGH on coarse grids (bands above); resolve the bending thickness with >= 4 voxels before quoting a number.
- LINEAR: no damping (no amplitudes/Q), no stress stiffening — a pre-tensioned or spinning part needs the (absent) preloaded-modal path; say so rather than quote.
- Isotropic: printed-layer anisotropy shifts real frequencies; treat results as as-designed, not as-printed truth.
- Mode IDENTIFICATION is on the caller: use the participation receipts (kinetic fraction + effective mass) exactly as the gates do; a bare sorted frequency list mixes bending/torsion/axial families.

## When to use
Resonance questions on printed parts: does a drybox roller or spool-holder bracket have a mode near the printer/motor excitation band; free-free spectra to compare against a physical tap test; mode-shape .npy -> GridField grade law to stiffen where a troublesome mode moves most. First frequency also feeds shipping/vibration sanity for assemblies.

Run: `ACE_PYTHON tools/analyzers/ace_modal_runner.py job.json` · prove: `python3 tools/tests/test_ace_modal_buckling.py`
