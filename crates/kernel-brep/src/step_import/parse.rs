// Copyright (c) LMCAD. Licensed under the MIT License.

//! The ISO-10303-21 physical-file reader: [`Value`]s, [`Entity`] instances, the
//! recursive-descent [`Cursor`] over one instance body, statement splitting, and
//! the two whole-file [`parse`] entry points (strict and lenient) plus the header's
//! asserted uncertainty.

use std::collections::HashMap;

use super::StepError;

/// A parsed STEP parameter value.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Value {
	Real(f64),
	Int(i64),
	Str(String),
	/// An enumeration like `.T.` or `.PLANE.`, stored without the dots.
	Enum(String),
	/// A `#N` entity reference.
	Ref(u32),
	List(Vec<Value>),
	/// An inline typed record `NAME(args)`.
	Typed(String, Vec<Value>),
	/// `$` (unset) or `*` (derived).
	Null,
}

impl Value {
	pub(crate) fn as_ref(&self) -> Option<u32> {
		match self {
			Value::Ref(r) => Some(*r),
			_ => None,
		}
	}
	pub(crate) fn as_list(&self) -> Option<&[Value]> {
		match self {
			Value::List(v) => Some(v),
			_ => None,
		}
	}
	pub(crate) fn as_real(&self) -> Option<f64> {
		match self {
			Value::Real(r) => Some(*r),
			Value::Int(i) => Some(*i as f64),
			_ => None,
		}
	}
	pub(crate) fn as_int(&self) -> Option<i64> {
		match self {
			Value::Int(i) => Some(*i),
			_ => None,
		}
	}
	pub(crate) fn as_str(&self) -> Option<&str> {
		match self {
			Value::Str(s) => Some(s),
			_ => None,
		}
	}
}

/// One `#N = NAME(args);` instance.
pub(crate) struct Entity {
	pub(crate) name: String,
	pub(crate) args: Vec<Value>,
}

// --- Parser ------------------------------------------------------------------

/// Cursor over the bytes of a single instance body for recursive value parsing.
struct Cursor<'a> {
	s: &'a [u8],
	i: usize,
}

impl<'a> Cursor<'a> {
	fn new(s: &'a str) -> Self {
		Cursor { s: s.as_bytes(), i: 0 }
	}
	fn peek(&self) -> Option<u8> {
		self.s.get(self.i).copied()
	}
	fn skip_ws(&mut self) {
		while let Some(c) = self.peek() {
			if c.is_ascii_whitespace() {
				self.i += 1;
			} else {
				break;
			}
		}
	}

	/// Parse one value at the cursor.
	fn value(&mut self) -> Result<Value, StepError> {
		self.skip_ws();
		match self.peek() {
			None => Err(StepError::Parse("unexpected end of value".into())),
			Some(b'#') => {
				self.i += 1;
				let n = self.uint()?;
				Ok(Value::Ref(n))
			}
			Some(b'\'') => Ok(Value::Str(self.string()?)),
			Some(b'.') => Ok(Value::Enum(self.enumeration()?)),
			Some(b'(') => Ok(Value::List(self.list()?)),
			Some(b'$') | Some(b'*') => {
				self.i += 1;
				Ok(Value::Null)
			}
			Some(c) if c == b'+' || c == b'-' || c.is_ascii_digit() => self.number(),
			Some(c) if c.is_ascii_alphabetic() || c == b'_' => {
				let name = self.ident();
				self.skip_ws();
				if self.peek() == Some(b'(') {
					Ok(Value::Typed(name, self.list()?))
				} else {
					// A bare keyword constant (rare); treat as an enum-like token.
					Ok(Value::Enum(name))
				}
			}
			Some(c) => Err(StepError::Parse(format!("unexpected character '{}'", c as char))),
		}
	}

	/// Parse a parenthesised, comma-separated list (cursor on `(`).
	fn list(&mut self) -> Result<Vec<Value>, StepError> {
		debug_assert_eq!(self.peek(), Some(b'('));
		self.i += 1;
		let mut out = Vec::new();
		loop {
			self.skip_ws();
			match self.peek() {
				Some(b')') => {
					self.i += 1;
					return Ok(out);
				}
				None => return Err(StepError::Parse("unterminated list".into())),
				_ => {
					out.push(self.value()?);
					self.skip_ws();
					// `,` separates list items; `)` ends; anything else is the next
					// space-separated record of a complex instance `(A() B() C())`.
					match self.peek() {
						Some(b',') => self.i += 1,
						Some(b')') | None => {}
						_ => {}
					}
				}
			}
		}
	}

	fn uint(&mut self) -> Result<u32, StepError> {
		let start = self.i;
		while matches!(self.peek(), Some(c) if c.is_ascii_digit()) {
			self.i += 1;
		}
		if self.i == start {
			return Err(StepError::Parse("expected integer after '#'".into()));
		}
		std::str::from_utf8(&self.s[start..self.i])
			.ok()
			.and_then(|t| t.parse().ok())
			.ok_or_else(|| StepError::Parse("bad entity id".into()))
	}

	fn string(&mut self) -> Result<String, StepError> {
		debug_assert_eq!(self.peek(), Some(b'\''));
		self.i += 1;
		let mut out = String::new();
		loop {
			match self.peek() {
				None => return Err(StepError::Parse("unterminated string".into())),
				Some(b'\'') => {
					// `''` is an escaped single quote.
					if self.s.get(self.i + 1) == Some(&b'\'') {
						out.push('\'');
						self.i += 2;
					} else {
						self.i += 1;
						return Ok(out);
					}
				}
				Some(c) => {
					out.push(c as char);
					self.i += 1;
				}
			}
		}
	}

	fn enumeration(&mut self) -> Result<String, StepError> {
		debug_assert_eq!(self.peek(), Some(b'.'));
		self.i += 1;
		let start = self.i;
		while matches!(self.peek(), Some(c) if c != b'.') {
			self.i += 1;
		}
		if self.peek() != Some(b'.') {
			return Err(StepError::Parse("unterminated enumeration".into()));
		}
		let s = std::str::from_utf8(&self.s[start..self.i]).unwrap_or("").to_string();
		self.i += 1;
		Ok(s)
	}

	fn ident(&mut self) -> String {
		let start = self.i;
		while matches!(self.peek(), Some(c) if c.is_ascii_alphanumeric() || c == b'_') {
			self.i += 1;
		}
		std::str::from_utf8(&self.s[start..self.i]).unwrap_or("").to_string()
	}

	fn number(&mut self) -> Result<Value, StepError> {
		let start = self.i;
		let mut real = false;
		if matches!(self.peek(), Some(b'+') | Some(b'-')) {
			self.i += 1;
		}
		while let Some(c) = self.peek() {
			match c {
				b'0'..=b'9' => self.i += 1,
				b'.' => {
					real = true;
					self.i += 1;
				}
				b'e' | b'E' => {
					real = true;
					self.i += 1;
					if matches!(self.peek(), Some(b'+') | Some(b'-')) {
						self.i += 1;
					}
				}
				_ => break,
			}
		}
		let t = std::str::from_utf8(&self.s[start..self.i]).unwrap_or("");
		if real {
			t.parse::<f64>().map(Value::Real).map_err(|_| StepError::Parse(format!("bad real '{t}'")))
		} else {
			t.parse::<i64>().map(Value::Int).map_err(|_| StepError::Parse(format!("bad integer '{t}'")))
		}
	}
}

/// Split the file into top-level `;`-terminated statements, ignoring `;` inside
/// `'…'` strings and `/* … */` comments.
fn statements(text: &str) -> Vec<String> {
	let b = text.as_bytes();
	let mut out = Vec::new();
	let mut cur = String::new();
	let mut i = 0;
	let mut in_str = false;
	while i < b.len() {
		let c = b[i];
		if in_str {
			cur.push(c as char);
			if c == b'\'' {
				if b.get(i + 1) == Some(&b'\'') {
					cur.push('\'');
					i += 2;
					continue;
				}
				in_str = false;
			}
			i += 1;
		} else if c == b'/' && b.get(i + 1) == Some(&b'*') {
			// Skip a block comment.
			i += 2;
			while i + 1 < b.len() && !(b[i] == b'*' && b[i + 1] == b'/') {
				i += 1;
			}
			i += 2;
		} else if c == b'\'' {
			in_str = true;
			cur.push('\'');
			i += 1;
		} else if c == b';' {
			out.push(cur.trim().to_string());
			cur.clear();
			i += 1;
		} else {
			cur.push(c as char);
			i += 1;
		}
	}
	out
}

/// Parse the whole file into an instance map (`#N → Entity`). Strict: the first
/// malformed instance statement fails the parse.
pub(crate) fn parse(text: &str) -> Result<HashMap<u32, Entity>, StepError> {
	parse_with(text, false).map(|(map, _)| map)
}

/// [`parse`] with an optional **lenient** mode for the tolerant importer: a
/// malformed instance statement is skipped and reported as `(statement head,
/// reason)` instead of failing the whole file. Strict mode (`lenient = false`)
/// returns the first malformed statement as the error and an empty issue list.
/// A parsed entity graph plus the `(statement head, reason)` of every statement
/// the lenient parse skipped.
pub(crate) type ParsedFile = (HashMap<u32, Entity>, Vec<(String, String)>);

pub(crate) fn parse_with(text: &str, lenient: bool) -> Result<ParsedFile, StepError> {
	let mut map = HashMap::new();
	let mut issues: Vec<(String, String)> = Vec::new();
	for stmt in statements(text) {
		// Only `#N = …` instance statements carry geometry.
		let Some(rest) = stmt.strip_prefix('#') else { continue };
		let Some(eq) = rest.find('=') else { continue };
		let head: String = stmt.chars().take(40).collect();
		let id: u32 = match rest[..eq].trim().parse() {
			Ok(id) => id,
			Err(_) if lenient => {
				issues.push((head, "bad entity id".into()));
				continue;
			}
			Err(_) => return Err(StepError::Parse(format!("bad id in '{stmt}'"))),
		};
		let body = rest[eq + 1..].trim();
		let mut cur = Cursor::new(body);
		let parsed = match cur.value() {
			Ok(v) => v,
			Err(e) => {
				let e = match e {
					StepError::Parse(m) => StepError::Parse(format!("{m} in `{body}`")),
					other => other,
				};
				if lenient {
					issues.push((head, e.to_string()));
					continue;
				}
				return Err(e);
			}
		};
		match parsed {
			Value::Typed(name, args) => {
				map.insert(id, Entity { name, args });
			}
			// A complex instance `#N=(A(..)B(..))` parses as a list of typed records;
			// keep the records under a synthetic name so lookups by sub-type still work.
			Value::List(items) => {
				map.insert(id, Entity { name: "_COMPLEX".into(), args: items });
			}
			_ => {} // non-entity assignment — ignore
		}
	}
	Ok((map, issues))
}

/// The file's asserted geometric **uncertainty** (mm): the largest
/// `UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(x), …)` in the entity graph —
/// the "maximum model space distance between geometric entities at asserted
/// connectivities" every AP203/AP214 producer writes into its representation
/// context. `None` when the file states none (a bare fragment).
pub(crate) fn file_uncertainty(ents: &HashMap<u32, Entity>) -> Option<f64> {
	ents.values()
		.filter(|e| e.name == "UNCERTAINTY_MEASURE_WITH_UNIT")
		.filter_map(|e| {
			e.args.iter().find_map(|v| match v {
				Value::Typed(n, a) if n == "LENGTH_MEASURE" => a.first().and_then(Value::as_real),
				_ => None,
			})
		})
		.filter(|u| u.is_finite() && *u > 0.0)
		.fold(None, |acc: Option<f64>, u| Some(acc.map_or(u, |a| a.max(u))))
}
