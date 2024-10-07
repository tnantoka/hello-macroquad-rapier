# hello-macroquad-rapier

Basic physics example using [macroquad](https:://github.com/not-fl3/macroquad) and [rapier](https://github.com/dimforge/rapier).

https://tnantoka.github.io/hello-macroquad-rapier/

## PC

```
$ cargo run
```

## WASM

```
$ cargo build --target wasm32-unknown-unknown --release
$ mv target/wasm32-unknown-unknown/release/hello-macroquad-rapier.wasm docs

$ serve docs
```

## Acknowledgements