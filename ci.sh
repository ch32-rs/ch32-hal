#!/bin/bash

set -ex
set -o pipefail

for d in $(find examples -depth 1 -type d); do
    (cd $d && cargo build --release)
done
