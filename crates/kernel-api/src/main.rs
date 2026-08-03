// Copyright (c) LMCAD. Licensed under the MIT License.

//! `kernel-api` CLI: execute a JSON program, or an `.lmcasm` assembly pipeline,
//! against the hybrid kernel.
//!
//! ```text
//! kernel-api run program.json [--out-dir DIR]
//! kernel-api asm assembly.lmcasm [--base-dir DIR] [--out-dir DIR]
//!                                [--tol MM] [--voxel MM] [--window MM]
//! ```
//!
//! The JSON [`kernel_api::Report`] is printed to stdout (always — even for a
//! file that cannot be read or parsed); the exit code is 0 iff every op/step
//! succeeded. Usage errors print to stderr and exit 2.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use kernel_api::{run_assembly, run_program_with_input_base, AsmOptions, ErrorKind, Report};

const USAGE: &str = "usage: kernel-api run <program.json> [--out-dir DIR]\n       kernel-api asm <assembly.lmcasm> [--base-dir DIR] [--out-dir DIR] [--tol MM] [--voxel MM] [--window MM]";

/// Print the report as pretty JSON on stdout and map it to the process exit code.
fn finish(report: Report) -> ExitCode {
	// A report of plain serializable data cannot fail to serialize.
	println!("{}", serde_json::to_string_pretty(&report).expect("report serialization"));
	if report.ok {
		ExitCode::SUCCESS
	} else {
		ExitCode::FAILURE
	}
}

/// A usage error: message + usage on stderr, exit 2.
fn usage_error(message: &str) -> ExitCode {
	eprintln!("{message}\n{USAGE}");
	ExitCode::from(2)
}

/// `kernel-api run …`: parse flags and execute the JSON program.
fn cmd_run(args: &[String]) -> ExitCode {
	let mut program_path: Option<PathBuf> = None;
	let mut out_dir = PathBuf::from(".");
	let mut rest = args.iter();
	while let Some(arg) = rest.next() {
		if arg == "--out-dir" {
			match rest.next() {
				Some(dir) => out_dir = PathBuf::from(dir),
				None => return usage_error("--out-dir requires a directory argument"),
			}
		} else if program_path.is_none() {
			program_path = Some(PathBuf::from(arg));
		} else {
			return usage_error(&format!("unexpected argument '{arg}'"));
		}
	}
	let Some(program_path) = program_path else {
		return usage_error("run: missing <program.json>");
	};

	let text = match std::fs::read_to_string(&program_path) {
		Ok(t) => t,
		Err(e) => {
			return finish(Report::program_failure(ErrorKind::Io, format!("cannot read program '{}': {e}", program_path.display())));
		}
	};
	if let Err(e) = std::fs::create_dir_all(&out_dir) {
		return finish(Report::program_failure(ErrorKind::Io, format!("cannot create out dir '{}': {e}", out_dir.display())));
	}
	// Relative `load_part` paths resolve against the program file's own
	// directory, so a program is relocatable (matching how a `.lmcasm` resolves
	// its `path` sources); outputs still land under --out-dir.
	let input_base = program_path.parent().map(Path::to_path_buf).unwrap_or_else(|| PathBuf::from("."));
	finish(run_program_with_input_base(&text, &out_dir, &input_base))
}

/// `kernel-api asm …`: parse flags and execute the assembly pipeline.
fn cmd_asm(args: &[String]) -> ExitCode {
	let mut asm_path: Option<PathBuf> = None;
	let mut out_dir = PathBuf::from(".");
	let mut opts = AsmOptions::default();
	let mut rest = args.iter();
	while let Some(arg) = rest.next() {
		match arg.as_str() {
			"--out-dir" => match rest.next() {
				Some(dir) => out_dir = PathBuf::from(dir),
				None => return usage_error("--out-dir requires a directory argument"),
			},
			"--base-dir" => match rest.next() {
				Some(dir) => opts.base_dir = Some(PathBuf::from(dir)),
				None => return usage_error("--base-dir requires a directory argument"),
			},
			"--tol" | "--voxel" | "--window" => {
				let Some(value) = rest.next() else {
					return usage_error(&format!("{arg} requires a positive number (mm)"));
				};
				let Ok(v) = value.parse::<f64>() else {
					return usage_error(&format!("{arg}: '{value}' is not a number"));
				};
				if !(v.is_finite() && v > 0.0) {
					return usage_error(&format!("{arg} must be a positive number (mm), got {value}"));
				}
				match arg.as_str() {
					"--tol" => opts.tol = v,
					"--voxel" => opts.voxel = v,
					_ => opts.window = v,
				}
			}
			_ if asm_path.is_none() && !arg.starts_with("--") => asm_path = Some(PathBuf::from(arg)),
			_ => return usage_error(&format!("unexpected argument '{arg}'")),
		}
	}
	let Some(asm_path) = asm_path else {
		return usage_error("asm: missing <assembly.lmcasm>");
	};
	if let Err(e) = std::fs::create_dir_all(&out_dir) {
		return finish(Report::program_failure(ErrorKind::Io, format!("cannot create out dir '{}': {e}", out_dir.display())));
	}
	finish(run_assembly(&asm_path, &out_dir, &opts))
}

fn main() -> ExitCode {
	let args: Vec<String> = std::env::args().skip(1).collect();
	match args.first().map(String::as_str) {
		Some("run") => cmd_run(&args[1..]),
		Some("asm") => cmd_asm(&args[1..]),
		_ => usage_error("expected a subcommand: run | asm"),
	}
}
