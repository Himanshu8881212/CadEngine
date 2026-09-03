// Copyright (c) LMCAD. Licensed under the MIT License.

//! Exact pose evaluators for the reduction drive trains this repo builds.
//!
//! The kinematic simulators (`cyclo26_sim.rs`, `harmonic26_sim.rs`,
//! `planetary26_sim.rs` example binaries on the drive branches, now parked in
//! `legacy/kernel-model-examples/`) originally
//! hand-rolled this math, and two real bugs shipped mid-development because
//! of it:
//!
//! - the textbook stepped-planet install phase `β·(1 + S/Pa)` mis-clocked an
//!   11T planet stage by ⅔ of a tooth (see [`EpicyclicTrain::poses`] for the
//!   full story);
//! - the strain-wave tangential displacement shipped with the wrong sign,
//!   violating ring inextensibility (see [`StrainWaveTrain::deformation`]).
//!
//! This module is the single source of truth: carrier/planet/ring pose
//! formulas and the install-phase conventions live next to the code that
//! computes them, so a simulator and the printed parts can never drift apart
//! on convention. Angles are radians; `theta` is always the INPUT shaft
//! angle. All evaluators are pure functions — no state, no panics (degenerate
//! zero-tooth trains yield non-finite `f64`s, and [`EpicyclicTrain::validate_assembly`]
//! rejects them up front).

use std::f64::consts::{PI, TAU};

/// A Wolfrom (3K) compound epicyclic train: sun `S` drives planets `Pa`,
/// planets `Pa` roll in the grounded ring `R1`, and a second gear `Pb` —
/// rigidly stepped on each planet shaft — drives the output ring `R2`.
///
/// This is the PLAN-26 architecture: `EpicyclicTrain { sun_teeth: 12,
/// ring1_teeth: 36, planet_a_teeth: 12, planet_b_teeth: 11, ring2_teeth: 39,
/// n_planets: 3 }` gives exactly 26:1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EpicyclicTrain {
	/// Sun gear tooth count `S` (the input).
	pub sun_teeth: usize,
	/// Grounded ring tooth count `R1` (meshes the `Pa` planet band).
	pub ring1_teeth: usize,
	/// First planet band tooth count `Pa` (meshes sun and ring1).
	pub planet_a_teeth: usize,
	/// Second planet band tooth count `Pb` (stepped rigid with `Pa`, meshes ring2).
	pub planet_b_teeth: usize,
	/// Output ring tooth count `R2` (meshes the `Pb` planet band).
	pub ring2_teeth: usize,
	/// Number of equally spaced planets `n`.
	pub n_planets: usize,
}

/// Exact pose set of every member of an [`EpicyclicTrain`] at one input angle.
///
/// All angles are absolute (radians, about the central axis for `carrier` /
/// `sun` / `ring2`, about each planet's own axis for [`PlanetPose::spin`]) and
/// INCLUDE the install phases, so a solid built at its install phase and then
/// rotated to the pose angle is exactly meshed.
#[derive(Clone, Debug, PartialEq)]
pub struct EpicyclicPoses {
	/// Carrier angle `φc = θ·S/(S+R1)`.
	pub carrier: f64,
	/// Sun absolute angle `π/S + θ` — install phase plus input rotation.
	pub sun: f64,
	/// The sun's install phase `π/S` (half a tooth pitch), returned
	/// separately so a builder can clock the printed sun once and drive it
	/// with raw `θ` afterwards.
	pub sun_install_phase: f64,
	/// Output ring absolute angle `θ · ω_out` (see [`EpicyclicTrain::ratio`]).
	pub ring2: f64,
	/// The output ring's install phase: exactly `0.0` (probe-verified on
	/// PLAN-26; kept as a field so the convention is explicit at call sites).
	pub ring2_install_phase: f64,
	/// One entry per planet, `j = 0..n_planets`.
	pub planets: Vec<PlanetPose>,
}

/// Pose of one planet of an [`EpicyclicTrain`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlanetPose {
	/// Orbital azimuth of the planet centre about the sun axis: `βj + φc`.
	pub azimuth: f64,
	/// Absolute spin about the planet's own axis: `βj + spin(θ)` with
	/// `spin(θ) = φc − (S/Pa)·(θ − φc)`.
	pub spin: f64,
	/// Install phase `ψ0 = βj = 2πj/n` — the spin at `θ = 0`. See
	/// [`EpicyclicTrain::poses`] for why this is `βj` and NOT the textbook
	/// `βj·(1 + S/Pa)`.
	pub install_phase: f64,
}

impl EpicyclicTrain {
	/// Check the tooth-count divisibility conditions that let `n_planets`
	/// identical planets be installed at equal azimuths `βj = 2πj/n` with the
	/// uniform install-phase convention of [`Self::poses`].
	///
	/// Three conditions, each with its own failure message:
	/// - `S + R1` must be even — the sun/ring1 stage centres a planet tooth on
	///   a half-pitch grid, so an odd sum leaves the sun's half-pitch phase
	///   `π/S` unable to close the mesh on both sides of a planet;
	/// - `(S + R1) % n == 0` — the classic equal-spacing closure condition for
	///   the sun/ring1 stage (planet azimuth `2πj/n` must land on a repeat of
	///   the combined sun+ring tooth pattern);
	/// - `R2 % n == 0` — the second-stage rings must repeat under rotation by
	///   `2π/n` so every stepped planet sees an identical ring2 phase.
	pub fn validate_assembly(&self) -> Result<(), String> {
		let (s, r1, r2, n) = (self.sun_teeth, self.ring1_teeth, self.ring2_teeth, self.n_planets);
		if n == 0 || s == 0 || self.planet_a_teeth == 0 || self.planet_b_teeth == 0 || r1 == 0 || r2 == 0 {
			return Err(format!(
				"degenerate train: every tooth count and n_planets must be nonzero (got S={s}, R1={r1}, Pa={}, Pb={}, R2={r2}, n={n})",
				self.planet_a_teeth, self.planet_b_teeth
			));
		}
		if (s + r1) % 2 != 0 {
			return Err(format!(
				"S + R1 = {s} + {r1} = {} is odd: the sun carries a half-pitch install phase π/S, and an odd sun+ring1 tooth sum leaves the sun and ring1 flanks half a pitch out of step on opposite sides of a planet — the first stage cannot close",
				s + r1
			));
		}
		if (s + r1) % n != 0 {
			return Err(format!(
				"(S + R1) = {} is not divisible by n_planets = {n}: equally spaced planets at βj = 2πj/{n} would each see a different sun/ring1 mesh phase — the classic epicyclic assembly condition (S+R) % n == 0 fails",
				s + r1
			));
		}
		if r2 % n != 0 {
			return Err(format!(
				"R2 = {r2} is not divisible by n_planets = {n}: ring2's tooth pattern does not repeat under the planet spacing 2π/{n}, so the stepped planets (clocked identically to their A bands) cannot all mesh ring2 at the same phase"
			));
		}
		Ok(())
	}

	/// Output ratio of the Wolfrom train (input revs per output rev).
	///
	/// With the input on the sun, ring1 grounded and ring2 the output, per
	/// unit sun angle: carrier rate `ωc = S/(S+R1)`, stage-2 coupling
	/// `k = S·Pb/(Pa·R2)`, output rate `ω_out = ωc·(1+k) − k`. Returns
	/// `1/ω_out`. PLAN-26 (12/36, 12→11, 39) gives exactly 26.
	pub fn ratio(&self) -> f64 {
		let s = self.sun_teeth as f64;
		let wc = s / (s + self.ring1_teeth as f64);
		let k = s * self.planet_b_teeth as f64 / (self.planet_a_teeth as f64 * self.ring2_teeth as f64);
		1.0 / (wc * (1.0 + k) - k)
	}

	/// Exact absolute pose of every member at input (sun) angle `theta`.
	///
	/// - carrier `φc = θ·S/(S+R1)`;
	/// - planet `j`: orbital azimuth `βj + φc` with `βj = 2πj/n`, absolute
	///   spin `ψj = βj + spin(θ)` where `spin(θ) = φc − (S/Pa)·(θ − φc)`
	///   (rolling on the sun: `ωp = ωc − (S/Pa)(ωs − ωc)`, `ωs = 1`);
	/// - ring2 at `θ·ω_out` (see [`Self::ratio`]).
	///
	/// # Install-phase convention — and why the textbook formula is WRONG here
	///
	/// Planets install at `ψ0 = βj`, the SUN carries a half-pitch phase
	/// `π/S`, and ring2 installs at zero phase.
	///
	/// The textbook stepped-planet install formula `β·(1 + S/Pa)` is WRONG:
	/// it holds the sun fixed while rolling the planet around it to azimuth
	/// `β`, which re-phases the sun mesh — harmless for a simple planet
	/// (its spin phase is absorbed tooth-by-tooth), but a stepped planet's B
	/// band is rigidly clocked to its A band, so the spurious extra roll
	/// becomes a real mis-mesh in stage 2. The correct phase comes from
	/// rotating the WHOLE meshed assembly rigidly by `β`: a rigid rotation
	/// preserves every mesh, and it maps the grounded members onto themselves
	/// exactly when `β` is a multiple of every ring's angular pitch. The
	/// [`Self::validate_assembly`] divisibility conditions guarantee that for
	/// the symmetric trains this repo builds (`n` divides `S`, `R1` and `R2`
	/// — e.g. PLAN-26's 12/36/39 with `n = 3`), so planet `j` installs with
	/// spin exactly `βj`.
	///
	/// This bug SHIPPED mid-development as a ⅔-tooth mis-clock of an 11T
	/// stage: for PLAN-26's planet 1, `β·(1 + S/Pa) − β = 2π/3`, which modulo
	/// the 11T pitch `2π/11` is exactly ⅔ of a tooth — enough to jam stage 2
	/// everywhere. The regression is pinned in `tests/kinematics.rs`.
	pub fn poses(&self, theta: f64) -> EpicyclicPoses {
		let s = self.sun_teeth as f64;
		let pa = self.planet_a_teeth as f64;
		let phic = theta * s / (s + self.ring1_teeth as f64);
		let spin = phic - (s / pa) * (theta - phic);
		let wc = s / (s + self.ring1_teeth as f64);
		let k = s * self.planet_b_teeth as f64 / (pa * self.ring2_teeth as f64);
		let w_out = wc * (1.0 + k) - k;
		let n = self.n_planets;
		let planets = (0..n)
			.map(|j| {
				let beta = TAU * j as f64 / n as f64;
				PlanetPose { azimuth: beta + phic, spin: beta + spin, install_phase: beta }
			})
			.collect();
		EpicyclicPoses {
			carrier: phic,
			sun: PI / s + theta,
			sun_install_phase: PI / s,
			ring2: theta * w_out,
			ring2_install_phase: 0.0,
			planets,
		}
	}

	/// Ratio of a plain single-stage planetary (sun in, carrier out, ring
	/// grounded): `1 + R/S`.
	pub fn simple_ratio(sun: usize, ring: usize) -> f64 {
		1.0 + ring as f64 / sun as f64
	}

	/// The [`Self::poses`] angles turned into **rigid instance poses** — the
	/// bridge from the kinematic evaluators to [`crate::Assembly`] /
	/// [`crate::Instance`] placements (previously the caller had to hand-roll
	/// this trigonometry, which is exactly how the two shipped clocking bugs
	/// happened).
	///
	/// Geometry convention: the train is centred on the world origin with +z
	/// as the rotation axis; `module_mm` scales tooth counts into radii, so a
	/// planet's centre orbits at `module·(S + Pa)/2` mm. Each returned pose is
	/// world = rotation(member angle about z) ∘ translation(orbit) applied to
	/// a member modelled AT THE ORIGIN in its install orientation: place a sun
	/// / planet / ring solid modelled at install phase and these poses mesh it
	/// exactly at input angle `theta` (radians).
	pub fn instance_poses(&self, theta: f64, module_mm: f64) -> EpicyclicInstancePoses {
		use kernel_core::math::{DAffine3, DVec3};
		let p = self.poses(theta);
		let rot_z = |ang: f64| DAffine3::from_axis_angle(DVec3::Z, ang);
		let r_orbit = module_mm * (self.sun_teeth + self.planet_a_teeth) as f64 / 2.0;
		let planets = p
			.planets
			.iter()
			.map(|pl| {
				let centre = DVec3::new(r_orbit * pl.azimuth.cos(), r_orbit * pl.azimuth.sin(), 0.0);
				// Spin about the planet's OWN axis, then carry it to its orbit station.
				DAffine3::from_translation(centre) * rot_z(pl.spin)
			})
			.collect();
		EpicyclicInstancePoses {
			sun: rot_z(p.sun),
			carrier: rot_z(p.carrier),
			ring2: rot_z(p.ring2),
			planets,
			orbit_radius_mm: r_orbit,
		}
	}
}

/// Rigid member poses of an [`EpicyclicTrain`] at one input angle — the
/// assembly-ready form of [`EpicyclicPoses`] (see
/// [`EpicyclicTrain::instance_poses`] for the geometric convention).
#[derive(Clone, Debug, PartialEq)]
pub struct EpicyclicInstancePoses {
	/// Sun pose (rotation about +z, at the origin).
	pub sun: kernel_core::math::DAffine3,
	/// Carrier pose (rotation about +z, at the origin).
	pub carrier: kernel_core::math::DAffine3,
	/// Output-ring pose (rotation about +z, at the origin).
	pub ring2: kernel_core::math::DAffine3,
	/// One pose per planet: spin about its own axis, translated to its orbit
	/// station (azimuth from [`PlanetPose::azimuth`]).
	pub planets: Vec<kernel_core::math::DAffine3>,
	/// The planet orbit radius `module·(S + Pa)/2` used for the translations (mm).
	pub orbit_radius_mm: f64,
}

/// A two-lobe strain-wave (harmonic) drive: flexspline with `flex_teeth`
/// teeth inside a grounded circular spline with `flex_teeth + 2` teeth, wave
/// generator as the input. HARM-26 is `StrainWaveTrain { flex_teeth: 52 }`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StrainWaveTrain {
	/// Flexspline tooth count `F` (the circular spline has `F + 2`).
	pub flex_teeth: usize,
}

impl StrainWaveTrain {
	/// Circular-spline tooth count: `flex_teeth + 2` (two-lobe wave).
	pub fn circ_teeth(&self) -> usize {
		self.flex_teeth + 2
	}

	/// Reduction ratio, wave-generator in / flexspline out, circular spline
	/// grounded: `F/2` (the output runs BACKWARDS relative to the input —
	/// see [`Self::flex_creep`] for the sign).
	pub fn ratio(&self) -> f64 {
		self.flex_teeth as f64 / 2.0
	}

	/// Flexspline output rotation at wave angle `theta`: `ψ = −2θ/F`.
	/// Negative — each wave revolution walks the flexspline BACK by its
	/// two-tooth deficit against the grounded circular spline.
	pub fn flex_creep(&self, theta: f64) -> f64 {
		-2.0 * theta / self.flex_teeth as f64
	}

	/// Neutral-line displacement of the flexspline wall at material angle
	/// `phi` for wave angle `theta` and radial amplitude `w0`: returns
	/// `(radial, tangential)` in the units of `w0`.
	///
	/// Standard inextensible thin-ring strain-wave kinematics:
	/// `w = w0·cos 2(φ−θ)` radial, `v = −(w0/2)·sin 2(φ−θ)` tangential —
	/// the tangential field is forced by arc-length conservation of the
	/// neutral line, `dv/dφ = −w`.
	///
	/// The tangential SIGN was a shipped bug: `v = +(w0/2)·sin 2(φ−θ)`
	/// violates inextensibility (it stretches the neutral line and bunches
	/// the teeth toward the lobes instead of away), and the first HARM-26
	/// tooth sweep interfered everywhere until the sign was flipped. The
	/// regression is pinned in `tests/kinematics.rs`.
	pub fn deformation(&self, w0: f64, phi: f64, theta: f64) -> (f64, f64) {
		let a = 2.0 * (phi - theta);
		(w0 * a.cos(), -(w0 / 2.0) * a.sin())
	}
}

/// A cycloidal drive: `lobes`-lobed disc(s) on an eccentric cam inside a
/// `lobes + 1` pin ring, ring grounded, disc creep as the output. CYCLO-26 is
/// `CycloidTrain { lobes: 26 }`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CycloidTrain {
	/// Disc lobe count `L` (the pin ring has `L + 1` pins).
	pub lobes: usize,
}

impl CycloidTrain {
	/// Reduction ratio cam-in / disc-out with the pin ring grounded: `L`.
	pub fn ratio(&self) -> f64 {
		self.lobes as f64
	}

	/// Disc spin at cam angle `theta`: `−θ/L` — the disc creeps backwards one
	/// lobe pitch per cam revolution. Geometry enforces this exactly (a
	/// deliberately wrong creep jams within one revolution; the drive
	/// simulators gate on it).
	pub fn disc_creep(&self, theta: f64) -> f64 {
		-theta / self.lobes as f64
	}

	/// Spin phase of the SECOND disc: `−π/L`.
	///
	/// A balanced two-disc drive mounts the second disc on the cam half a
	/// turn out (cam phase `π`), so its pose at cam angle `θ` is the first
	/// disc's pose evaluated at `θ + π`: spin `−(θ+π)/L = disc_creep(θ) −
	/// π/L`. This constant is that `−π/L` offset.
	pub fn second_disc_phase(&self) -> f64 {
		-PI / self.lobes as f64
	}
}
