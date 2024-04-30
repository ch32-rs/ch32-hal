#!/bin/bash

set -ex
set -o pipefail

for d in $(find examples -type d -depth 1); do
    (cd $d && cargo build --release)
done
