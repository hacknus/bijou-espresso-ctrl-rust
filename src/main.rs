//#![deny(unsafe_code)]
#![no_main]
#![no_std]
// For allocator
#![feature(lang_items)]
#![feature(alloc_error_handler)]

use cortex_m::asm;
use cortex_m_rt::exception;
use cortex_m_rt::{entry, ExceptionFrame};
use panic_halt as _;
use arrform::{arrform, ArrForm};
use stm32f4xx_hal::otg_fs::{USB};
use stm32f4xx_hal::{
    pac::{self, Interrupt},
    gpio::{self, Edge, Input},
    prelude::*,
};

use freertos_rust::*;
use core::alloc::Layout;


use crate::i2c::i2c1_init;
use crate::led::LED;
use crate::spi::spi2_init;
use crate::usb::{usb_init, usb_print, usb_println, usb_read};
use crate::intrpt::{G_BUTTON};

#[path = "devices/led.rs"]
mod led;
mod usb;
mod i2c;
mod spi;
mod commands;
mod intrpt;


#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;



#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .require_pll48clk()
        .pclk1(24.MHz())
        .pclk2(24.MHz())
        .freeze();

    let mut delay = dp.TIM1.delay_us(&clocks);

    // initialize ports
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpioe = dp.GPIOE.split();
    let gpiod = dp.GPIOD.split();

    // initialize pins
    let cs_2 = gpiod.pd12.into_push_pull_output();
    let cs_1 = gpiod.pd13.into_push_pull_output();
    let mut en_2 = gpioc.pc6.into_push_pull_output();
    let mut en_1 = gpioe.pe12.into_push_pull_output();
    let mut x_end_l = gpioe.pe10.into_push_pull_output();
    let mut x_end_r = gpioe.pe11.into_push_pull_output();
    let mut y_end_l = gpiob.pb11.into_push_pull_output();
    let mut y_end_r = gpiob.pb12.into_push_pull_output();
    x_end_r.set_low();
    y_end_r.set_low();
    x_end_l.set_low();
    y_end_l.set_low();

    // initialize leds
    let mut stat_led = LED::new(gpioe.pe2.into_push_pull_output());
    let mut fault_1_led = LED::new(gpioe.pe3.into_push_pull_output());
    let mut fault_2_led = LED::new(gpioe.pe4.into_push_pull_output());

    // initialize switch
    let mut sw = gpiob.pb8.into_floating_input();
    let mut syscfg = dp.SYSCFG.constrain();
    sw.make_interrupt_source(&mut syscfg);
    sw.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    sw.enable_interrupt(&mut dp.EXTI);

    // initialize usb
    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into_alternate(),
        pin_dp: gpioa.pa12.into_alternate(),
        hclk: clocks.hclk(),
    };
    delay.delay(100.millis());
    unsafe {
        usb_init(usb);
        cortex_m::peripheral::NVIC::unmask(Interrupt::OTG_FS);
        // Enable the external interrupt in the NVIC by passing the button interrupt number
        cortex_m::peripheral::NVIC::unmask(sw.interrupt());
    }
    usb_println("usb set up ok");

    let scl = gpiob.pb6;
    let sda = gpiob.pb7;
    // initialize i2c
    i2c1_init(dp.I2C1, scl, sda, &clocks);

    let sclk = gpiob.pb13;
    let sdo = gpiob.pb14;
    let sdi = gpiob.pb15;
    // initialize spi
    spi2_init(dp.SPI2, sclk, sdo, sdi, &clocks);

    // Now that button is configured, move button into global context
    cortex_m::interrupt::free(|cs| {
        G_BUTTON.borrow(cs).replace(Some(sw));
    });

    stat_led.on();

    for i in 0..=3 {
        delay.delay(1000.millis());
        match i {
            0 => { stat_led.on() }
            1 => { fault_1_led.on() }
            2 => { fault_2_led.on() }
            _ => {
                stat_led.off();
                fault_1_led.off();
                fault_2_led.off();
            }
        }
        usb_println(arrform!(64, "T-{}", 3-i).as_str());
    }

    usb_println("boot up ok");

    for _ in 0..=4 {
        delay.delay(200.millis());
        stat_led.toggle();
    }

    // infinite loop; just so we don't leave this stack frame
    loop {
        let mut message_bytes = [0; 1024];
        if usb_read(&mut message_bytes) {
            match core::str::from_utf8(&message_bytes) {
                Ok(cmd) => {
                    usb_print("received: ");
                    usb_println(cmd);
                }
                Err(_) => {}
            }
        }

        // delay 100 ms
        delay.delay(100.millis());
        // one run takes approximately 0.05s

        // toggle LED (one blink means two HK packets have been sent)
        stat_led.toggle();
        usb_println("cycle completed");
    }
}


#[exception]
unsafe fn DefaultHandler(_irqn: i16) {
// custom default handler
// irqn is negative for Cortex-M exceptions
// irqn is positive for device specific (line IRQ)
// set_led(true);(true);
// panic!("Exception: {}", irqn);
}

#[exception]
unsafe fn HardFault(_ef: &ExceptionFrame) -> ! {
// Blink 3 times long when exception occures
    //delay_n(10);
    for _ in 0..3 {
        // set_led(true);
        // delay_n(1000);
        // set_led(false);
        // delay_n(555);
    }
    loop {}
}

// define what happens in an Out Of Memory (OOM) condition
#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    //set_led(true);
    asm::bkpt();
    loop {}
}

#[no_mangle]
fn vApplicationStackOverflowHook(pxTask: FreeRtosTaskHandle, pcTaskName: FreeRtosCharPtr) {
    asm::bkpt();
}