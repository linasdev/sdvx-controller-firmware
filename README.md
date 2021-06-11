# Sound Voltex Controller Firmware
This repository contains firmware for an open-sorce [Sound Voltex](https://en.wikipedia.org/wiki/Sound_Voltex) controller.

# Features
- Utilizes the STM32F103C8T microcontroller
- USB 2.0 12 Mbit/s (Full Speed)
- 1 millisecond polling rate
- Interrupt based rotary encoder inputs (less errors)
- Default keyboard mappings for [unnamed-sdvx-clone](https://github.com/Drewol/unnamed-sdvx-clone)

# Getting Started

## Dependencies
To build and flash this project you will need:

- OpenOCD. [Installation instructions](http://openocd.org/getting-openocd/).
- Rust toolchain. [Installation instructions](https://www.rust-lang.org/learn/get-started).
- `rust-std` components for the `thumbv7m-none-eabi` target. Run:
    ```
    $ rustup target add thumbv7m-none-eabi
    ```
- `cargo-binutils`. Run:
    ```
    $ cargo install cargo-binutils
    ```

## Building
To build this project, run:
```
$ cargo build --release
```

## Flashing
To flash the built firmware, run:
```
$ cargo objcopy --release -- -O binary ./target/thumbv7m-none-eabi/release/sdvx-controller-firmware.bin
$ openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "init; reset halt; stm32f1x mass_erase 0; flash write_bank 0 ./target/thumbv7m-none-eabi/release/sdvx-controller-firmware.bin ; reset run; shutdown;"
```

# License
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
