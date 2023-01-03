use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::{
    pac::I2C1,
    i2c::{Mode as i2cMode, Error, I2c},
    gpio::{PB6, PB7},
    rcc::Clocks,
    prelude::*,
};

pub static I2C_BUS: Mutex<RefCell<Option<I2c<I2C1, (PB6, PB7)>>>> =
    Mutex::new(RefCell::new(None));

pub fn i2c1_init(i2c1: I2C1, scl: PB6, sda: PB7, clocks: &Clocks) {
    let i2c: I2c<I2C1, (PB6, PB7)> = i2c1.i2c(
        (scl, sda),
        i2cMode::Standard {
            frequency: 100.kHz(),
        },
        clocks,
    );
    cortex_m::interrupt::free(|cs| {
        *I2C_BUS.borrow(cs).borrow_mut() = Some(i2c);
    });
}

pub fn i2c_read(address: u8, buffer: &mut [u8]) -> Result<(), Error> {
    cortex_m::interrupt::free(|cs| {
        match I2C_BUS.borrow(cs).borrow_mut().as_mut() {
            None => { Err(Error::Timeout) }
            Some(i2c) => {
                i2c.read(address, buffer)
            }
        }
    })
}

pub fn i2c_write(address: u8, bytes: &[u8]) -> Result<(), Error> {
    cortex_m::interrupt::free(|cs| {
        match I2C_BUS.borrow(cs).borrow_mut().as_mut() {
            None => { Err(Error::Timeout) }
            Some(i2c) => {
                i2c.write(address, bytes)
            }
        }
    })
}