# Sound Voltex Controller Firmware
This repository contains firmware for an open-sorce [Sound Voltex](https://en.wikipedia.org/wiki/Sound_Voltex) controller.

# Features
- Utilizes the STM32F103C8T microcontroller
- USB 2.0 12 Mbit/s (Full Speed)
- 1 millisecond polling rate
- Uses [rotary-encoder-hal](https://github.com/leshow/rotary-encoder-hal) for rotary encoder input.
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
- `binutils`. [Installation instructions](https://www.gnu.org/software/binutils/).

## Building
To build this project, run:
```
$ cargo build --release
```

## Flashing
To flash the firmware, run: 
```
$ ./flash.sh
```

# License
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
