#!/bin/sh
cargo build --release && cargo test --release && cargo release patch --no-publish --execute
