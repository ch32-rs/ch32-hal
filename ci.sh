#!/bin/bash

set -ex
set -o pipefail

for d in $(ls -1 examples); do
    (cd $d && cargo build --release)
done
