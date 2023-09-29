### Bijou Espresso Machine Control Software (Rust Code)

This board controls the Bijou Espresso Machine.  
It is based around an STM32F405 microcontroller that reads out 5 PT100 RTDs (2 wire) using 5 MAX31865 ADCs.  
The temperature values are printed out over USB (CDC virtual COM Port) and shown on the SSD1306 OLED display.  
Two heating elements are controlled (SSR40 solid state relays) with PWM signals.  
Additionally a Buzzer and multiple LEDs can be controlled (PWM).  

The software uses freertos-rust (wrapper for rust) and thus requires nightly to compile.  
The embedded-graphics crate is used for the display.

Connect the board using a ST-Link V3 (with TagConnect) to a USB port on the computer. Be sure to power the board with an additional USB-C connector. (ST-Link does not provide power)  
First start the openocd server in a terminal/console window:
```openocd -f interface/stlink-v2-1.cfg -f target/stm32f4x.cfg```   
Flash/Run the code on the hardware using:  
```cargo run --package bijou-espresso-ctrl-rust --bin bijou-espresso-ctrl-rust --target thumbv7em-none-eabihf --release```  

To set up your system, be sure to follow [this guide](https://docs.rust-embedded.org/discovery/f3discovery/03-setup/index.html)!  
Don't forget to install the [ARM GNU toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads).  
Make sure you add did 
```rustup install nightly``` and
```rustup target add thumbv7em-none-eabihf --toolchain nightly``` to add the target to the nightly toolchain.  

TODO:
- implement SPI flash chip in software
