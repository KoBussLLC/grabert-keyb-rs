# Rust Firmware for Grabert

Rust firmware for the [Grabert Keyboard](https://github.com/KoBussLLC/grabert-hardware) using the [Keyberon](https://github.com/TeXitoi/keyberon) crate by TeXitoi

### Helpful Commands

Build:
``` sh
cargo build --release
```

Firmware Size: Sitting around 36KB
``` sh
cargo-size --bin grabert-keyb-rs --release
```

OpenOCD:
``` sh
openocd -f "interface/stlink-v2.cfg" -f "target/stm32f0x.cfg" "./target/thumbv6m-none-eabi/release/grabert-keyb-rs"
```

### License
MIT