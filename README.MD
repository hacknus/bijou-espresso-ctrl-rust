### Bijou Espresso Machine Control Software (Rust Code)

This board controls the Bijou Espresso Machine.  
It is based around an STM32F405 microcontroller that reads out 5 PT100 RTDs (2 wire) using 5 MAX31865 ADCs.  
The temperature values are printed out over USB (CDC virtual COM Port) and shown on the SSD1306 OLED display.  
Two heating elements are controlled (SSR40 solid state relays) with PWM signals.  
Additionally a Buzzer and multiple LEDs can be controlled (PWM).

The software uses freertos-rust (wrapper for rust) and thus requires nightly to compile.  
The embedded-graphics crate is used for the display.

## Flashing the hardware (USB):

Setup a python environment in the project folder and install `pyserial`.

Update the `scripts/enter_bootloader.py` script to use the correct serial port for your system.

Connect the board using USB and then run

```
cargo run --release
```

in your python environment.

## Flashing the hardware (ST-Link debugger):

Create a `.cargo/config.toml` file according to this template:

```toml
[target.thumbv7em-none-eabihf]
runner = "arm-none-eabi-gdb -q -x openocd.gdb"
# uncomment these for other setups
# runner = "gdb-multiarch -q"
# runner = "gdb -q"

rustflags = [
    "-C", "link-arg=-Tlink.x"
]

[build]
target = "thumbv7em-none-eabihf"
```

specify your runner (depends on platform).

Connect the board using an ST-Link V3 (with TagConnect) to a USB port on the computer. Be sure to power the board with
an additional USB-C connector. (ST-Link does not provide power)  
First start the openocd server in a terminal/console window:
```openocd -f interface/stlink.cfg -f target/stm32f4x.cfg```   
Flash/Run the code on the hardware using:  
```cargo run --release```

## Notes on flashing/building

To set up your system, be sure to
follow [this rust embedded stm32 guide](https://docs.rust-embedded.org/discovery/f3discovery/03-setup/index.html)!  
Don't forget to install the [ARM GNU toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads).

Now that the `rust-toolchain.toml` exists, the two steps below might not be necessary.  
Make sure you did
```rustup install nightly``` and
```rustup target add thumbv7em-none-eabihf --toolchain nightly``` to add the target to the nightly toolchain.

