

# rustup component add llvm-tools-preview
rm -f ./*.profraw
export RUSTC_BOOTSTRAP=1
export CARGO_INCREMENTAL=0
export RUSTFLAGS="-Zinstrument-coverage -Zprofile -Ccodegen-units=1 -Copt-level=0 -Clink-dead-code -Coverflow-checks=off -Zpanic_abort_tests -Cpanic=abort"
export RUSTDOCFLAGS="-Cpanic=abort"
# export RUSTFLAGS="-Zinstrument-coverage"

cargo build
export LLVM_PROFILE_FILE="./tests-%p-%m.profraw"
cargo test

grcov . -s . --binary-path ./target/debug/ -t html --branch --ignore-not-existing -o ./target/debug/coverage/
