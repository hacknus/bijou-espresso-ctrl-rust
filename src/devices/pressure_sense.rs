use core::cell::RefCell;

use crate::usb::usb_println;
use arrform::{arrform, ArrForm};
use cortex_m::interrupt::Mutex;
use freertos_rust::{CurrentTask, Duration};
use stm32f4xx_hal::adc::Adc;
use stm32f4xx_hal::dma::{PeripheralToMemory, Stream0, Transfer};
use stm32f4xx_hal::pac::interrupt;
use stm32f4xx_hal::pac::{ADC1, DMA2};
// use crate::usb::usb_println;

type DMATransfer = Transfer<Stream0<DMA2>, 0, Adc<ADC1>, PeripheralToMemory, &'static mut [u16; 3]>;

pub static G_XFR: Mutex<RefCell<Option<DMATransfer>>> = Mutex::new(RefCell::new(None));
pub static G_CURRENT: Mutex<RefCell<Option<f32>>> = Mutex::new(RefCell::new(None));
pub static G_ADC_BUF: Mutex<RefCell<Option<[u16; 3]>>> = Mutex::new(RefCell::new(None));

pub static mut ADC_MEMORY: [u16; 3] = [0u16; 3];

const OFFSET: f32 = 0.006;

pub fn read_pressure() -> Option<f32> {
    let mut voltage = 0.0;
    cortex_m::interrupt::free(|cs| match G_CURRENT.borrow(cs).borrow_mut().as_mut() {
        None => {}
        Some(sampled_voltage) => voltage = *sampled_voltage / 3.3 * 5.0,
    });
    // add some delay
    CurrentTask::delay(Duration::ms(2));
    // trigger a new conversion (is this really needed?)
    cortex_m::interrupt::free(|cs| {
        if let Some(transfer) = G_XFR.borrow(cs).borrow_mut().as_mut() {
            transfer.start(|adc| {
                adc.start_conversion();
            });
        };
    });
    // transform to pressure according to datasheet:
    // https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/2247/114991178_Web.pdf
    usb_println(arrform!(64, "Pressure Voltage: {:}", voltage).as_str());

    if (0.5..=4.5).contains(&voltage) {
        Some((voltage - 0.5) * 12.0 / 4.0)
    } else {
        None
    }
}

#[interrupt]
#[allow(non_snake_case)]
fn DMA2_STREAM0() {
    cortex_m::interrupt::free(|cs| {
        if let Some(xfer) = G_XFR.borrow(cs).borrow_mut().as_mut() {
            unsafe {
                if let Ok((buffer, _)) = xfer.next_transfer(&mut ADC_MEMORY) {
                    let sample_to_millivolts = xfer.peripheral().make_sample_to_millivolts();
                    G_CURRENT.borrow(cs).replace(Some(
                        sample_to_millivolts(buffer[1]) as f32 / 1000.0 - OFFSET,
                    ));
                    G_ADC_BUF.borrow(cs).replace(Some(*buffer));
                }
            }
        } else {
            //println!("DMA1_CH1 IRQ: ERR: no xfer").unwrap();
        }
    });
}
