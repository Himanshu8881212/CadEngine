# Security Policy

## Supported versions

LMCAD is currently **alpha/experimental**. Until a signed release is published,
only the latest protected `main` commit receives security fixes. Do not use the
project for safety-critical decisions or expose Studio directly to an untrusted
network.

## Reporting a vulnerability

Please use this repository's **GitHub private vulnerability reporting / Security
Advisory** feature. Do not open a public issue containing an exploit, credential,
or runner/network detail.

Include:

- affected commit and component;
- reproducible steps or a minimal proof of concept;
- impact and required attacker access;
- suggested mitigation, if known.

Maintainers should acknowledge a report within 7 days and coordinate disclosure
after a fix is available. Never include real credentials or proprietary models.

## Security boundaries

- Pull-request code must run only on GitHub-hosted, disposable runners. Persistent
  self-hosted runners may execute only protected-branch commits or explicitly
  selected protected `main` code.
- Studio binds to loopback by default. A non-loopback bind requires both
  `CADCODE_ALLOW_REMOTE=1` and `CADCODE_API_TOKEN`; a reverse proxy with TLS,
  request limits, and network isolation is still required.
- CAD/analyzer jobs are untrusted inputs. Export paths are confined, symlink
  traversal is rejected, optimizer expressions use a non-executable AST grammar,
  and subprocess results must exit zero and satisfy the receipt contract.
- HTTP deadlines do not pre-empt Rust CPU instructions. Timed-out work remains
  counted against `CADCODE_COMPUTE_CONCURRENCY` until it exits. Public or
  multi-tenant deployments must additionally use OS/container worker isolation,
  CPU/memory quotas, and process-level termination.
- Manufacturing output is accepted only after strict in-memory and serialized
  round-trip validation. A successful write is not a dimensional or process
  certification.

## Release requirements

A release must come from a clean protected commit with green release gates,
including Rust tests/Clippy/formatting, analyzer contracts and pinned validation,
web build/audit, dependency audits, strict gallery artifact checks, install smoke
tests, SBOMs, checksums, and GitHub artifact provenance attestations. The Python
environment must match `tools/requirements-analysis.lock`; a dirty checkout cannot
produce a `validated` receipt. The FEA/modal/buckling solvers are in-tree at
`tools/analyzers/physics/` (Apache-2.0, see its `NOTICE`), so the commit being
released pins them — the separate `tools/ACE_REVISION` pin was retired 2026-09-04.
