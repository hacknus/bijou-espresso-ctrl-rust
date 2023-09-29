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
        const MAX31865_RREF: u16 = 4000; // Reference resistor
        const MAX31865_FACTOR: u16 = 32768; // 2^15 used for data to resistance conversion
        if let Ok(raw_resistance) = self.get_raw() {
            Some((raw_resistance as f32 * MAX31865_RREF as f32) / MAX31865_FACTOR as f32)
            // offset for cable resistance
        } else {
            None
        }
    }

    // Deprecated, old
    //
    // pub fn get_temperature(&mut self, spi: &mut Spi<SPI1, (PA5, PA6, PA7)>) -> Option<f32> {
    //     const MAX31865_ALPHA: f32 = 0.03851; // PT-1000 temperature coefficient
    //     if let Some(resistance) = self.get_resistance(spi) {
    //         Some(((resistance / 100.0) - 1.0) / MAX31865_ALPHA + 273.15)
    //     } else {
    //         None
    //     }
    // }

    // source of this approximation: http://www.mosaic-industries.com/embedded-systems/microcontroller-projects/temperature-measurement/platinum-rtd-sensors/resistance-calibration-table
    // attention: might not be accurate below -200 °C
    pub fn get_temperature(&mut self) -> Option<f32> {
        const C0: f32 = -245.19;
        const C1: f32 = 2.5293;
        const C2: f32 = -0.066046;
        const C3: f32 = 4.0422E-3;
        const C4: f32 = -2.0697E-6;
        const C5: f32 = -0.025422;
        const C6: f32 = 1.6883E-3;
        const C7: f32 = -1.3601E-6;
        if let Some(r) = self.get_resistance() {
            let temperature = C0
                + (r / 10.0 * (C1 + r / 10.0 * (C2 + r / 10.0 * (C3 + C4 * r / 10.0))))
                    / (1.0 + r / 10.0 * (C5 + r / 10.0 * (C6 + C7 * r / 10.0)));
            Some(temperature + 273.15)
        } else {
            None
        }
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
