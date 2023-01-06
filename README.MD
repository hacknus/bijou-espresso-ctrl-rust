### Bijou Espresso Machine Control Software

This board controls the Bijou Espresso Machine.


Connect the board using a ST-Link V3 (with TagConnect) to a USB port on the computer. Be sure to power the board with an additional USB-C connector. (ST-Link does not provide power)  
First start the openocd server in a terminal/console window:
```openocd -f interface/stlink-v2-1.cfg -f target/stm32f4x.cfg```   
Flash/Run the code on the hardware using:  
```cargo +nightly run --package xy-stepper-ctrl --bin xy-stepper-ctrl --target thumbv7em-none-eabihf --release```  
note: release is important for USB to work reliably!  
note: nightly is important because of the FreeRTOS wrapper.

To set up your system, be sure to follow [this guide](https://docs.rust-embedded.org/discovery/f3discovery/03-setup/index.html)!  
Make sure you add did 
```rustup install nightly``` and
```rustup target att thumbv7em-none-eabihg --toolchain nightly``` to add the target to the nightly toolchain.