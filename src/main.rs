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
use stm32f4xx_hal::dwt::DwtExt;
use usb_device::UsbError;

use crate::i2c::i2c1_init;
use crate::led::LED;
use crate::spi::spi2_init;
use crate::usb::{usb_init, usb_print, usb_println, usb_read};
use crate::max31865::MAX31865;
use crate::pid::PID;
use crate::tasks::{blink_led_1_task, blink_led_2_task, print_usb_task};

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{
        Circle, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StrokeAlignment, Triangle,
    },
    text::{Alignment, Text},
    mock_display::MockDisplay,
};

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use stm32f4xx_hal::{
    pac::I2C1,
    i2c::{Mode as i2cMode, Error, I2c},
    gpio::{PB6, PB7},
    rcc::Clocks,
    prelude::*,
};
use tinybmp::Bmp;
use embedded_graphics::{image::Image, pixelcolor::Rgb565, prelude::*};


#[path = "devices/led.rs"]
mod led;
#[path = "devices/max31865.rs"]
mod max31865;
mod usb;
mod i2c;
mod spi;
mod commands;
mod intrpt;
mod tasks;
mod pid;


#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;


#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::peripheral::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .hclk(48.MHz())
        .require_pll48clk()
        .pclk1(24.MHz())
        .pclk2(24.MHz())
        .freeze();

    let dwt = cp.DWT.constrain(cp.DCB, &clocks);
    let mut timer = dp.TIM2.counter_ms(&clocks);

    let mut delay = dp.TIM1.delay_us(&clocks);
    delay.delay(100.millis());  // apparently required for USB to set up properly...

    // initialize ports
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpioe = dp.GPIOE.split();
    let gpiod = dp.GPIOD.split();

    // initialize pins
    let cs_5 = gpiod.pd14.into_push_pull_output();
    let cs_4 = gpiod.pd10.into_push_pull_output();
    let cs_3 = gpiod.pd11.into_push_pull_output();
    let cs_2 = gpiod.pd12.into_push_pull_output();
    let cs_1 = gpiod.pd13.into_push_pull_output();
    let mut bldc_en = gpioa.pa8.into_push_pull_output();
    let mut bldc_dir = gpioa.pa7.into_push_pull_output();

    // initialize leds
    let mut stat_led = LED::new(gpioe.pe2.into_push_pull_output());
    let mut fault_1_led = LED::new(gpioe.pe13.into_push_pull_output());
    let mut fault_2_led = LED::new(gpioe.pe14.into_push_pull_output());

    // initialize switch
    let mut sw = gpiob.pb0.into_floating_input();

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
    }
    stat_led.on();
    // usb_println("usb set up ok");

    let scl = gpiob.pb6;
    let sda = gpiob.pb7;
    // initialize i2c
    let i2c: I2c<I2C1, (PB6, PB7)> = dp.I2C1.i2c(
        (scl, sda),
        i2cMode::Standard {
            frequency: 100.kHz(),
        },
        &clocks,
    );
    // i2c1_init(dp.I2C1, scl, sda, &clocks);

    let interface = I2CDisplayInterface::new(i2c);

    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let sclk = gpiob.pb13;
    let sdo = gpiob.pb14;
    let sdi = gpiob.pb15;
    // initialize spi
    spi2_init(dp.SPI2, sclk, sdo, sdi, &clocks);

    // let mut max31865_1 = MAX31865::new(cs_1, 0.0, 2);
    // let mut max31865_2 = MAX31865::new(cs_2, 0.0, 2);
    // let mut max31865_3 = MAX31865::new(cs_3, 0.0, 2);
    // let mut max31865_4 = MAX31865::new(cs_4, 0.0, 2);
    // let mut max31865_5 = MAX31865::new(cs_5, 0.0, 2);

    // max31865_1.init();
    // max31865_2.init();
    // max31865_3.init();
    // max31865_4.init();
    // max31865_5.init();

    usb_println("boot up ok");

    // Task::new()
    //     .name("TEMPERATURE ADC TASK")
    //     .stack_size(128)
    //     .priority(TaskPriority(2))
    //     .start(move || {
    //         loop {
    //             let t1 = 0.0; //max31865_1.get_temperature();
    //             let t2 = 0.0; //max31865_2.get_temperature();
    //             let t3 = 0.0; //max31865_3.get_temperature();
    //             let t4 = 0.0; //max31865_4.get_temperature();
    //             let t5 = 0.0; // max31865_5.get_temperature();
    //
    //             usb_println(arrform!(128, "{:.2}, {:.2}, {:.2}, {:.2}, {:.2}", t1, t2, t3, t4, t5).as_str());
    //         }
    //     }).unwrap();

    // Task::new()
    //     .name("PID TASK")
    //     .stack_size(256)
    //     .priority(TaskPriority(3))
    //     .start(move || {
    //         let mut pid = PID::new();
    //         loop {
    //             //pid.calculate(25.0, timer.now().ticks());
    //             freertos_rust::CurrentTask::delay(Duration::ms(1000));
    //         }
    //     }).unwrap();

    Task::new()
        .name("DISPLAY TASK")
        .stack_size(256)
        .priority(TaskPriority(2))
        .start(move || {
            loop {
                let bmp = Bmp::from_slice(include_bytes!("../rust.bmp")).expect("Failed to load BMP image");

                // The image is an RGB565 encoded BMP, so specifying the type as `Image<Bmp<Rgb565>>` will read
                // the pixels correctly
                let im: Image<Bmp<Rgb565>> = Image::new(&bmp, Point::new(32, 0));

                // We use the `color_converted` method here to automatically convert the RGB565 image data into
                // BinaryColor values.
                im.draw(&mut display.color_converted()).unwrap();

                display.flush().unwrap();
                freertos_rust::CurrentTask::delay(Duration::ms(1000));
                let bmp = Bmp::from_slice(include_bytes!("../rust1.bmp")).expect("Failed to load BMP image");

                // The image is an RGB565 encoded BMP, so specifying the type as `Image<Bmp<Rgb565>>` will read
                // the pixels correctly
                let im: Image<Bmp<Rgb565>> = Image::new(&bmp, Point::new(32, 0));

                // We use the `color_converted` method here to automatically convert the RGB565 image data into
                // BinaryColor values.
                im.draw(&mut display.color_converted()).unwrap();

                display.flush().unwrap();
                freertos_rust::CurrentTask::delay(Duration::ms(1000));
            }
        }).unwrap();

    // Task::new()
    //     .name("IDLE LED TASK")
    //     .stack_size(128)
    //     .priority(TaskPriority(2))
    //     .start(move || {}).unwrap();

    Task::new()
        .name("stat LED")
        .stack_size(64)
        .priority(TaskPriority(2))
        .start(move || {
            blink_led_1_task(stat_led)
        }).unwrap();

    Task::new()
        .name("fault1 LED")
        .stack_size(64)
        .priority(TaskPriority(1))
        .start(move || {
            blink_led_2_task(fault_1_led)
        }).unwrap();

    Task::new()
        .name("USB TASK")
        .stack_size(128)
        .priority(TaskPriority(4))
        .start(move || {
            // print_usb_task()
            loop {
                 let timestamp =  timer.now().ticks();
                 usb_println(arrform!(64,"t = {:?}", timestamp).as_str());
                 freertos_rust::CurrentTask::delay(Duration::ms(1000));
             }
        }).unwrap();

    FreeRtosUtils::start_scheduler();
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