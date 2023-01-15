use arrform::{arrform, ArrForm};
use cortex_m::asm::delay;
use stm32f4xx_hal::{
    gpio::{Output, Pin},
    spi::Error,
    prelude::*,
};
use crate::spi::{spi_read, spi_write};
use crate::usb_println;

pub struct MAX31865<const PC: char, const NC: u8> {
    pub cs_pin: Pin<PC, NC, Output>,
    pub offset: f32,
    pub wires: u8,
}

impl<const PC: char, const NC: u8> MAX31865<PC, NC> {
    pub fn new(
        mut cs_pin: Pin<PC, NC, Output>, offset: f32, wires: u8) -> Self {
        cs_pin.set_high();
        MAX31865 {
            cs_pin,
            offset,
            wires,
        }
    }

    pub fn init(&mut self) {
        let mut data = 0;
        if self.wires == 3 {
            data = 0b11010000; // 3 wire
        } else {
            data = 0b11000000; // 2 or 4 wire
        }

        self.write(0x00, data).unwrap();
    }

    pub fn get_raw_resistance(&mut self) -> u16 {
        let mut buffer = [0; 2];
        self.read(0x01, &mut buffer).unwrap();
        let mut data: u16 = (buffer[0] as u16) << 8;
        data |= (buffer[1] as u16);
        data >>= 1;
        data
    }

    pub fn get_resistance(&mut self) -> f32 {
        const MAX31865_RREF: u16 = 400;                        // Reference resistor
        const MAX31865_FACTOR: u16 = 32768;                    // 2^15 used for data to resistance conversion
        let raw_resistance = self.get_raw_resistance();
        (raw_resistance as f32 * MAX31865_RREF as f32) / MAX31865_FACTOR as f32 - self.offset  // offset for cable resistance
    }

    pub fn get_temperature(&mut self) -> f32 {
        const MAX31865_ALPHA: f32 = 0.003851;                  // PT-100 temperature coefficient
        let resistance = self.get_resistance();
        ((resistance / 100.0) - 1.0) / MAX31865_ALPHA
    }

    fn read(&mut self, register: u8, value: &mut [u8]) -> Result<u8, Error> {
        spi_read(&mut self.cs_pin, register, value)
        // usb_println(arrform!(64, "read payload = {:?}", buffer).as_str());
    }

    fn write(&mut self, register: u8, value: u8) -> Result<u8, Error> {
        spi_write(&mut self.cs_pin, register, value)
    }
}