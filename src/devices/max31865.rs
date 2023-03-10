use stm32f4xx_hal::{
    pac::SPI2,
    spi::{Error, Spi},
    gpio::{Output, PB13, PB14, PB15, Pin},
    prelude::*,
};

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

    pub fn init(&mut self, spi: &mut Spi<SPI2, (PB13, PB14, PB15)>) -> Result<u8, Error> {
        let data = if self.wires == 3 {
            0b11010000 // 3 wire
        } else {
            0b11000000 // 2 or 4 wire
        };

        self.write(spi, 0x00, data)
    }

    pub fn get_raw_resistance(&mut self, spi: &mut Spi<SPI2, (PB13, PB14, PB15)>) -> Option<u16> {
        let mut buffer = [0; 3];  // first entry is register addr, next two will be data
        match self.read(spi, 0x01, &mut buffer) {
            Ok(_) => {
                let data: u16 = (buffer[1] as u16) << 8 | (buffer[2] as u16);
                Some(data >> 1)
            }
            Err(_) => {
                None
            }
        }
    }

    pub fn get_resistance(&mut self, spi: &mut Spi<SPI2, (PB13, PB14, PB15)>) -> Option<f32> {
        const MAX31865_RREF: u16 = 400;                        // Reference resistor
        const MAX31865_FACTOR: u16 = 32768;                    // 2^15 used for data to resistance conversion
        if let Some(raw_resistance) = self.get_raw_resistance(spi) {
            Some((raw_resistance as f32 * MAX31865_RREF as f32) / MAX31865_FACTOR as f32 - self.offset)  // offset for cable resistance
        } else {
            None
        }
    }

    pub fn get_temperature(&mut self, spi: &mut Spi<SPI2, (PB13, PB14, PB15)>) -> Option<f32> {
        const MAX31865_ALPHA: f32 = 0.003851;                  // PT-100 temperature coefficient
        if let Some(resistance) = self.get_resistance(spi) {
            Some(((resistance / 100.0) - 1.0) / MAX31865_ALPHA)
        } else {
            None
        }
    }

    fn read(&mut self, spi: &mut Spi<SPI2, (PB13, PB14, PB15)>, register: u8, buffer: &mut [u8]) -> Result<u8, Error> {
        buffer[0] = register;
        //usb_println(arrform!(64,"read cmd buffer {:?}", buffer).as_str());
        self.cs_pin.set_low();
        match spi.transfer(buffer) {
            Ok(_) => {
                // usb_println(arrform!(64,"read answer {:?}",r).as_str());
            }
            Err(err) => {
                // usb_println(arrform!(64, "spi failed to read = {:?}", err).as_str());
                return Err(err);
            }
        }
        self.cs_pin.set_high();
        // usb_println(arrform!(64, "read payload = {:?}", buffer).as_str());
        Ok(0)
    }

    fn write(&mut self, spi: &mut Spi<SPI2, (PB13, PB14, PB15)>, register: u8, payload: u8) -> Result<u8, Error> {
        let mut buffer: [u8; 2] = [register | 0x80, payload];
        self.cs_pin.set_low();
        match spi.transfer(&mut buffer) {
            Ok(_) => {
                // usb_println(arrform!(64,"write answer {:?}",r).as_str());
            }
            Err(err) => {
                // usb_println(arrform!(64, "spi failed to write = {:?}", err).as_str());
                return Err(err);
            }
        }
        self.cs_pin.set_high();
        Ok(0)
    }
}