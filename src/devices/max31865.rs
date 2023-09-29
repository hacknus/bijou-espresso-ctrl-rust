use embedded_hal::{
    blocking::spi::{Transfer, Write},
    digital::v2::OutputPin,
};

#[allow(unused)]
#[derive(PartialEq)]
pub enum Wires {
    TwoWire,
    ThreeWire,
    FourWire,
}

pub enum MAX31865Error<E> {
    RTDHighThreshold,
    RTDLowThreshold,
    RefInHigh,
    RefInLow,
    RTDInLow,
    OverUnderVoltage,
    SPIError(E),
    DefaultError,
}

pub struct MAX31865<SPI, CS> {
    spi: SPI,
    pub cs_pin: CS,
    pub wires: Wires,
}

impl<SPI, CS, E> MAX31865<SPI, CS>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    CS: OutputPin,
{
    pub fn new(spi: SPI, mut cs_pin: CS, wires: Wires) -> Self {
        cs_pin.set_high().ok();
        MAX31865 { spi, cs_pin, wires }
    }

    pub fn init(&mut self) -> Result<(), MAX31865Error<E>> {
        let data = match self.wires {
            Wires::TwoWire | Wires::FourWire => 0b11000000,
            Wires::ThreeWire => 0b11010000,
        };
        self.write(0x00, data)?;
        self.get_fault()
    }

    pub fn get_fault(&mut self) -> Result<(), MAX31865Error<E>> {
        let mut buffer = [0]; // first entry is register addr, next two will be data
        let f = self.read(0x07, &mut buffer).map(|_| buffer[0])?;
        if f & 0b10000000 == 0b10000000 {
            Err(MAX31865Error::RTDHighThreshold)
        } else if f & 0b01000000 == 0b01000000 {
            Err(MAX31865Error::RTDLowThreshold)
        } else if f & 0b00100000 == 0b00100000 {
            Err(MAX31865Error::RefInHigh)
        } else if f & 0b00010000 == 0b00010000 {
            Err(MAX31865Error::RefInLow)
        } else if f & 0b00001000 == 0b00001000 {
            Err(MAX31865Error::RTDInLow)
        } else if f & 0b000000100 == 0b000000100 {
            Err(MAX31865Error::OverUnderVoltage)
        } else {
            Ok(())
        }
    }

    pub fn get_raw(&mut self) -> Result<u16, MAX31865Error<E>> {
        let mut buffer = [0; 3]; // first entry is register addr, next two will be data
        match self.read(0x01, &mut buffer) {
            Ok(_) => {
                let data: u16 = (buffer[1] as u16) << 8 | (buffer[2] as u16);
                // if fault, then return Error
                if (data | 0b1) == 0b1 {
                    // let _ = self.init();
                    match self.get_fault() {
                        Ok(_) => Err(MAX31865Error::DefaultError),
                        Err(err) => Err(err),
                    }
                } else {
                    Ok(data >> 1)
                }
            }
            Err(err) => Err(err),
        }
    }

    pub fn get_resistance(&mut self) -> Option<f32> {
        const MAX31865_RREF: u16 = 400; // Reference resistor
        const MAX31865_FACTOR: u16 = 32768; // 2^15 used for data to resistance conversion
        if let Ok(raw_resistance) = self.get_raw() {
            Some((raw_resistance as f32 * MAX31865_RREF as f32) / MAX31865_FACTOR as f32)
            // offset for cable resistance
        } else {
            None
        }
    }

    pub fn get_temperature(&mut self) -> Option<f32> {
        const MAX31865_ALPHA: f32 = 0.003851; // PT-100 temperature coefficient
        self.get_resistance()
            .map(|resistance| ((resistance / 100.0) - 1.0) / MAX31865_ALPHA)
    }
    fn read(&mut self, register: u8, buffer: &mut [u8]) -> Result<(), MAX31865Error<E>> {
        buffer[0] = register;
        //usb_println(arrform!(64,"read cmd buffer {:?}", buffer).as_str());
        self.cs_pin.set_low().ok();
        match self.spi.transfer(buffer) {
            Ok(_) => {
                // usb_println(arrform!(64,"read answer {:?}",r).as_str());
            }
            Err(err) => {
                // usb_println(arrform!(64, "spi failed to read = {:?}", err).as_str());
                return Err(MAX31865Error::SPIError(err));
            }
        }
        self.cs_pin.set_high().ok();
        // usb_println(arrform!(64, "read payload = {:?}", buffer).as_str());
        Ok(())
    }

    fn write(&mut self, register: u8, payload: u8) -> Result<u8, MAX31865Error<E>> {
        let mut buffer: [u8; 2] = [register | 0x80, payload];
        self.cs_pin.set_low().ok();
        match self.spi.transfer(&mut buffer) {
            Ok(_) => {
                // usb_println(arrform!(64,"write answer {:?}",r).as_str());
            }
            Err(err) => {
                // usb_println(arrform!(64, "spi failed to write = {:?}", err).as_str());
                return Err(MAX31865Error::SPIError(err));
            }
        }
        self.cs_pin.set_high().ok();
        Ok(0)
    }
}
