// Copyright (c) LMCAD. Licensed under the MIT License.

//! # studio-tui — CAD Code, the terminal front-end for LMCAD Studio
//!
//! The `lmcad-tui` binary (see `main.rs`) is a thin dispatcher over these
//! modules:
//!
//! - [`client`] — blocking HTTP + SSE client for the Studio server (the
//!   `/api/run` report mirror, the `/api/chat` event protocol, auto-spawn).
//!   Pure of terminal concerns; unit-tested.
//! - [`headless`] — the `-p` (one-shot chat) and `--run` (work-order) modes:
//!   plain stdout, script/CI-friendly exit codes.
//! - [`tui`] — the ratatui full-screen app: conversation log, receipts pane,
//!   command prompt. Thin: all network work happens through [`client`] on
//!   worker threads.

pub mod client;
pub mod headless;
pub mod tui;
