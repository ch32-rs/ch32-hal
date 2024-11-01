#!/bin/bash
## on pull_requrest

set -euo pipefail

# Build with all features
cargo build --features ${CHIP},embassy,rt,memory-x --no-default-features --target ${TARGET}

# Build without embassy
cargo build --features ${CHIP},rt --no-default-features --target ${TARGET}

# Build without embassy and rt
cargo build --features ${CHIP} --no-default-features --target ${TARGET}
