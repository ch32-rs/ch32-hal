//! Helpers shared by the host-test `build.rs` scripts that rewrite production
//! driver source into `OUT_DIR`.

use std::fs;
use std::path::{Path, PathBuf};

pub fn read(path: &Path) -> String {
    fs::read_to_string(path).unwrap_or_else(|error| panic!("cannot read {}: {error}", path.display()))
}

pub fn canonical(path: &Path) -> PathBuf {
    path.canonicalize()
        .unwrap_or_else(|error| panic!("cannot resolve {}: {error}", path.display()))
}

/// Replace one production-source anchor, failing loudly when the source moves.
pub fn replace(source: String, from: &str, to: &str) -> String {
    assert_eq!(
        source.matches(from).count(),
        1,
        "production-source anchor moved (expected one match): {from:?}"
    );
    source.replace(from, to)
}

pub fn strip_inner_docs(source: String) -> String {
    source
        .lines()
        .skip_while(|line| line.is_empty() || line.starts_with("//!"))
        .collect::<Vec<_>>()
        .join("\n")
}
