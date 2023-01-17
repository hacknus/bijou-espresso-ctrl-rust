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
    i2c::{Mode as i2cMode, Error as I2CError, I2c},
    gpio::{PB6, PB7},
    rcc::Clocks,
    prelude::*,
};
use stm32f4xx_hal::{
    pac::SPI2,
    spi::{Error as SPIError, Spi, Phase, Polarity, Mode as spiMode},
    gpio::{Output, PB13, PB14, PB15, Pin},
    prelude::*,
};
use tinybmp::Bmp;
use embedded_graphics::{image::Image, pixelcolor::Rgb565, prelude::*};
use stm32f4xx_hal::timer::Channel;


#[path = "devices/led.rs"]
mod led;
#[path = "devices/max31865.rs"]
mod max31865;
mod usb;
mod i2c;
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
    let mut timer = dp.TIM5.counter_ms(&clocks);
    timer.start(2_678_400_000.millis()).unwrap(); // set the timeout to 31 days

    let mut delay = dp.TIM1.delay_us(&clocks);
    delay.delay(100.millis());  // apparently required for USB to set up properly...

    // initialize ports
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();
    let gpioe = dp.GPIOE.split();

    // initialize pins
    let mut cs_5 = gpiod.pd14.into_push_pull_output();
    let mut cs_4 = gpiod.pd10.into_push_pull_output();
    let mut cs_3 = gpiod.pd11.into_push_pull_output();
    let mut cs_2 = gpiod.pd12.into_push_pull_output();
    let mut cs_1 = gpiod.pd13.into_push_pull_output();
    let mut bldc_en = gpioa.pa8.into_push_pull_output();
    let mut bldc_dir = gpioa.pa7.into_push_pull_output();
    let mut buzz = gpioa.pa3.into_alternate();
    let mut led_dim = gpiob.pb1.into_alternate();
    let mut heater_1 = gpioc.pc6.into_alternate();
    let mut heater_2 = gpioc.pc7.into_alternate();

    // initialize leds
    let mut stat_led = LED::new(gpioe.pe2.into_push_pull_output());
    let mut fault_1_led = LED::new(gpioe.pe13.into_push_pull_output());
    let mut fault_2_led = LED::new(gpioe.pe14.into_push_pull_output());

    // initialize switch
    let mut sw = gpiob.pb0.into_floating_input();

    // initialize buzzer
    let mut buzz_pwm = dp.TIM2.pwm_hz(buzz, 2000.Hz(), &clocks);
    let max_duty = buzz_pwm.get_max_duty();
    buzz_pwm.set_duty(Channel::C4, max_duty / 2);


    // initialize pwm timer 3
    let (mut heater_1_pwm, mut heater_2_pwm, mut led_pwm) = dp.TIM3.pwm_hz((heater_1, heater_2, led_dim), 2000.Hz(), &clocks).split();

    // initialize dimming LED
    let max_duty = led_pwm.get_max_duty();
    led_pwm.set_duty(max_duty / 2);

    // initialize heater_1
    let max_duty = heater_1_pwm.get_max_duty();
    heater_1_pwm.set_duty(max_duty / 2);

    // initialize heater_2
    let max_duty = heater_2_pwm.get_max_duty();
    heater_2_pwm.set_duty(max_duty / 2);

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

    let interface = I2CDisplayInterface::new(i2c);
    let bmp = Bmp::from_slice(include_bytes!("../rust.bmp")).expect("Failed to load BMP image");
    let bmp_inv = Bmp::from_slice(include_bytes!("../rust1.bmp")).expect("Failed to load BMP image");

    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let sclk = gpiob.pb13;
    let sdo = gpiob.pb14;
    let sdi = gpiob.pb15;

    // initialize spi
    let mut spi: Spi<SPI2, (PB13, PB14, PB15)> = dp.SPI2.spi(
        (sclk, sdo, sdi),
        spiMode {
            polarity: Polarity::IdleHigh,
            phase: Phase::CaptureOnSecondTransition,
        },
        10.kHz(),
        &clocks,
    );
    delay.delay(1000.millis());
    usb_println("boot up ok");
    delay.delay(100.millis());

    Task::new()
        .name("TEMPERATURE ADC TASK")
        .stack_size(1024)
        .priority(TaskPriority(4))
        .start(move || {
            let mut max31865_1 = MAX31865::new(cs_1, 0.0, 2);
            let mut max31865_2 = MAX31865::new(cs_2, 0.0, 2);
            let mut max31865_3 = MAX31865::new(cs_3, 0.0, 2);
            let mut max31865_4 = MAX31865::new(cs_4, 0.0, 2);
            let mut max31865_5 = MAX31865::new(cs_5, 0.0, 2);

            max31865_1.init(&mut spi);
            max31865_2.init(&mut spi);
            max31865_3.init(&mut spi);
            max31865_4.init(&mut spi);
            max31865_5.init(&mut spi);

            loop {
                let t1 = max31865_1.get_temperature(&mut spi);
                let t2 = max31865_2.get_temperature(&mut spi);
                let t3 = max31865_3.get_temperature(&mut spi);
                let t4 = max31865_4.get_temperature(&mut spi);
                let t5 = max31865_5.get_temperature(&mut spi);

                usb_println(arrform!(128, "{:.2}, {:.2}, {:.2}, {:.2}, {:.2}", t1, t2, t3, t4, t5).as_str());
                freertos_rust::CurrentTask::delay(Duration::ms(100));
            }
        }).unwrap();

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
        .stack_size(1024)
        .priority(TaskPriority(2))
        .start(move || {
            loop {

                // The image is an RGB565 encoded BMP, so specifying the type as `Image<Bmp<Rgb565>>` will read
                // the pixels correctly
                let im: Image<Bmp<Rgb565>> = Image::new(&bmp, Point::new(32, 0));

                // We use the `color_converted` method here to automatically convert the RGB565 image data into
                // BinaryColor values.
                im.draw(&mut display.color_converted()).unwrap();

                display.flush().unwrap();
                freertos_rust::CurrentTask::delay(Duration::ms(1000));

                // The image is an RGB565 encoded BMP, so specifying the type as `Image<Bmp<Rgb565>>` will read
                // the pixels correctly
                let im: Image<Bmp<Rgb565>> = Image::new(&bmp_inv, Point::new(32, 0));

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

    // Task::new()
    //     .name("BUZZER TASK")
    //     .stack_size(512)
    //     .priority(TaskPriority(4))
    //     .start(move || {
    //         // print_usb_task()
    //         let tones = [
    //             ('c', 261.Hz()),
    //             ('d', 294.Hz()),
    //             ('e', 329.Hz()),
    //             ('f', 349.Hz()),
    //             ('g', 392.Hz()),
    //             ('a', 440.Hz()),
    //             ('b', 493.Hz()),
    //         ];
    //
    //         let tune = [
    //             ('c', 1),
    //             ('c', 1),
    //             ('g', 1),
    //             ('g', 1),
    //             ('a', 1),
    //             ('a', 1),
    //             ('g', 2),
    //             ('f', 1),
    //             ('f', 1),
    //             ('e', 1),
    //             ('e', 1),
    //             ('d', 1),
    //             ('d', 1),
    //             ('c', 2),
    //             (' ', 4),
    //         ];
    //         let tempo = 300_u32;
    //
    //         loop {
    //             // 1. Obtain a note in the tune
    //             for note in tune {
    //                 // 2. Retrieve the freqeuncy and beat associated with the note
    //                 for tone in tones {
    //                     // 2.1 Find a note match in the tones array and update frequency and beat variables accordingly
    //                     if tone.0 == note.0 {
    //                         // 3. Play the note for the desired duration (beats*tempo)
    //                         // 3.1 Adjust period of the PWM output to match the new frequency
    //                         buzz_pwm.set_period(tone.1);
    //                         // 3.2 Enable the channel to generate desired PWM
    //                         buzz_pwm.enable(Channel::C4);
    //                         // 3.3 Keep the output on for as long as required
    //                         freertos_rust::CurrentTask::delay(Duration::ms(note.1 * tempo));
    //                     } else if note.0 == ' ' {
    //                         // 2.2 if ' ' tone is found disable output for one beat
    //                         buzz_pwm.disable(Channel::C4);
    //                         freertos_rust::CurrentTask::delay(Duration::ms(tempo));
    //                     }
    //                 }
    //                 // 4. Silence for half a beat between notes
    //                 // 4.1 Disable the PWM output (silence)
    //                 buzz_pwm.disable(Channel::C4);
    //                 // 4.2 Keep the output off for half a beat between notes
    //                 freertos_rust::CurrentTask::delay(Duration::ms(tempo / 2));
    //
    //                 // 5. Go back to 1.
    //             }
    //             let timestamp = timer.now().ticks();
    //             // usb_println(arrform!(64,"t = {:?}", timestamp).as_str());
    //             freertos_rust::CurrentTask::delay(Duration::ms(1000));
    //         }
    //     }).unwrap();

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