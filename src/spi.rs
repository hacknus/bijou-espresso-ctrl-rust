use core::cell::RefCell;
use arrform::{arrform, ArrForm};
use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::{
    pac::SPI2,
    spi::{Error, Spi, Phase, Polarity, Mode as spiMode},
    gpio::{Output, PB13, PB14, PB15, Pin},
    rcc::Clocks,
    prelude::*,
};

use crate::usb_println;

pub static SPI_BUS: Mutex<RefCell<Option<Spi<SPI2, (PB13, PB14, PB15)>>>> =
    Mutex::new(RefCell::new(None));

pub fn spi2_init(spi2: SPI2, sclk: PB13, sdo: PB14, sdi: PB15, clocks: &Clocks) {
    let spi: Spi<SPI2, (PB13, PB14, PB15)> = spi2.spi(
        (sclk, sdo, sdi),
        spiMode {
            polarity: Polarity::IdleHigh,
            phase: Phase::CaptureOnSecondTransition,
        },
        10.kHz(),
        clocks,
    );
    cortex_m::interrupt::free(|cs| {
        *SPI_BUS.borrow(cs).borrow_mut() = Some(spi);
    });
}

pub fn spi_write<const P: char, const N: u8>(cs_pin: &mut Pin<P, N, Output>, register: u8, payload: u8) -> Result<u8, Error> {
    cortex_m::interrupt::free(|cs| {
        match SPI_BUS.borrow(cs).borrow_mut().as_mut() {
            None => { Err(Error::ModeFault) }
            Some(spi) => {
                cs_pin.set_low();
                let mut status_byte = 0;
                let mut buffer: [u8; 2] = [register | 0x80, payload];
                // usb_println(arrform!(64,"write buffer {:?}",buffer).as_str());
                match spi.transfer(&mut buffer) {
                    Ok(r) => {
                        //usb_println(arrform!(64,"write answer {:?}",r).as_str());
                        status_byte = r[0];
                    }
                    Err(err) => {
                        //usb_println(arrform!(64, "spi failed to write = {:?}", err).as_str());
                    }
                }
                cs_pin.set_high();
                Ok(status_byte)
            }
        }
    })
}

pub fn spi_read<const P: char, const N: u8>(cs_pin: &mut Pin<P, N, Output>, register: u8, buffer: &mut [u8]) -> Result<u8, Error> {
    buffer[0] = register;
    cortex_m::interrupt::free(|cs| {
        match SPI_BUS.borrow(cs).borrow_mut().as_mut() {
            None => { Err(Error::ModeFault) }
            Some(spi) => {
                cs_pin.set_low();
                //usb_println(arrform!(64,"write buffer {:?}",read_cmd).as_str());
                match spi.transfer(buffer) {
                    Ok(r) => {
                        //usb_println(arrform!(64,"read answer {:?}",r).as_str());
                    }
                    Err(err) => {
                        //usb_println(arrform!(64, "spi failed to read = {:?}", err).as_str());
                    }
                }
                cs_pin.set_high();
                Ok(0)
            }
        }
    })
}
