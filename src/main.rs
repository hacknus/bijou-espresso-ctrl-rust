//#![deny(unsafe_code)]
#![no_main]
#![no_std]
// For allocator
#![feature(lang_items)]
#![feature(alloc_error_handler)]

extern crate alloc;

use alloc::sync::Arc;
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
use core::f32::consts::PI;

use crate::led::LED;
use crate::usb::{usb_init, usb_print, usb_println, usb_read};
use crate::max31865::MAX31865;
use crate::pid::PID;
use micromath::F32Ext;

use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{
        Circle, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StrokeAlignment, Triangle,
    },
    text::{Alignment, Text},
    mock_display::MockDisplay,
    mono_font::{
        ascii::{FONT_10X20, FONT_6X10, FONT_6X12, FONT_9X15},
        MonoTextStyle, MonoTextStyleBuilder,
    },
    mono_font::iso_8859_5::{FONT_5X8},
};

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use stm32f4xx_hal::{
    pac::I2C1,
    i2c::{Mode as i2cMode, I2c},
    gpio::{PB6, PB7},
    prelude::*,
};
use stm32f4xx_hal::{
    pac::SPI2,
    spi::{Spi, Phase, Polarity, Mode as spiMode},
    gpio::{Output, PB13, PB14, PB15, Pin},
    prelude::*,
};
use tinybmp::Bmp;
use embedded_graphics::{image::Image, pixelcolor::Rgb565, prelude::*};
use stm32f4xx_hal::timer::Channel;
use crate::intrpt::{G_ENC_PIN_A, G_ENC_PIN_B, G_ENC_STATE};
use crate::utils::{Interface, PidData, MeasuredData, State, PumpData, LedState};


use devices::led;
use devices::max31865;
use crate::commands::{extract_command, send_housekeeping};

mod usb;
mod commands;
mod intrpt;
mod pid;
mod utils;
mod devices;


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
        .hclk(48.MHz())
        .require_pll48clk()
        .pclk1(24.MHz())
        .pclk2(24.MHz())
        .freeze();

    let mut tick_timer = dp.TIM5.counter_ms(&clocks);
    tick_timer.start(2_678_400_000.millis()).unwrap(); // set the timeout to 31 days

    let mut delay = dp.TIM1.delay_us(&clocks);
    delay.delay(100.millis());  // apparently required for USB to set up properly...

    // initialize ports
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();
    let gpioe = dp.GPIOE.split();

    // initialize pins
    let cs_5 = gpiod.pd14.into_push_pull_output();
    let cs_4 = gpiod.pd10.into_push_pull_output();
    let cs_3 = gpiod.pd11.into_push_pull_output();
    let cs_2 = gpiod.pd12.into_push_pull_output();
    let cs_1 = gpiod.pd13.into_push_pull_output();
    let mut bldc_en = gpioa.pa8.into_push_pull_output();
    let mut bldc_dir = gpioc.pc9.into_push_pull_output();
    bldc_dir.set_low();
    let water_low = false;  // TODO: implement water_low pin
    let buzz = gpioa.pa3.into_alternate();
    let led_dim = gpiob.pb1.into_alternate();
    let mut heater_1 = gpioc.pc6.into_push_pull_output();
    let mut heater_2 = gpioc.pc7.into_push_pull_output();
    let bldc_v = gpioc.pc8.into_alternate();

    let enc_pin_a = gpioe.pe11.into_floating_input();
    let mut enc_pin_b = gpioe.pe9.into_floating_input();
    let enc_pin_sw = gpioe.pe10.into_floating_input();
    let mut syscfg = dp.SYSCFG.constrain();
    enc_pin_b.make_interrupt_source(&mut syscfg);
    enc_pin_b.trigger_on_edge(&mut dp.EXTI, Edge::RisingFalling);
    enc_pin_b.enable_interrupt(&mut dp.EXTI);


    // initialize leds
    let mut stat_led = LED::new(gpioe.pe2.into_push_pull_output());
    let mut fault_1_led = LED::new(gpioe.pe13.into_push_pull_output());
    let mut fault_2_led = LED::new(gpioe.pe14.into_push_pull_output());

    // initialize switch
    let sw = gpiob.pb0.into_floating_input();

    // initialize buzzer
    let mut buzz_pwm = dp.TIM2.pwm_hz(buzz, 2000.Hz(), &clocks);
    let max_duty = buzz_pwm.get_max_duty();
    buzz_pwm.set_duty(Channel::C4, max_duty / 2);

    // initialize pwm timer 3
    let (mut bldc_pwm, mut led_pwm) = dp.TIM3.pwm_hz((bldc_v, led_dim), 10000.Hz(), &clocks).split();

    // initialize dimming LED
    let max_duty = led_pwm.get_max_duty();
    led_pwm.set_duty(max_duty / 2);

    // initialize bldc pwm
    let max_duty = bldc_pwm.get_max_duty();
    bldc_pwm.set_duty(max_duty / 2);

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
        cortex_m::peripheral::NVIC::unmask(enc_pin_b.interrupt());
    }
    // Now that button is configured, move button into global context
    cortex_m::interrupt::free(|cs| {
        G_ENC_PIN_A.borrow(cs).replace(Some(enc_pin_a));
    });
    // Now that button is configured, move button into global context
    cortex_m::interrupt::free(|cs| {
        G_ENC_PIN_B.borrow(cs).replace(Some(enc_pin_b));
    });

    stat_led.on();
    // usb_println("usb set up ok");

    let mut pos: isize = 0;

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
    let bmp = Bmp::from_slice(include_bytes!("../images/rust.bmp")).expect("Failed to load BMP image");
    // let bmp_inv = Bmp::from_slice(include_bytes!("../images/rust1.bmp")).expect("Failed to load BMP image");

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

    let led_state = LedState::Off;
    let led_state_container = Arc::new(Mutex::new(led_state).expect("Failed to create data guard mutex"));
    let led_state_container_main = led_state_container.clone();
    let led_state_container_led = led_state_container.clone();

    let state = State::Idle;
    let state_container = Arc::new(Mutex::new(state).expect("Failed to create data guard mutex"));
    let state_container_main = state_container.clone();
    let state_container_usb = state_container.clone();

    let temperature_data = MeasuredData::default();
    let temperature_data_container = Arc::new(Mutex::new(temperature_data).expect("Failed to create data guard mutex"));
    let temperature_data_container_display = temperature_data_container.clone();
    let temperature_data_container_adc = temperature_data_container.clone();
    let temperature_data_container_main = temperature_data_container.clone();
    let temperature_data_container_pid = temperature_data_container.clone();
    let temperature_data_container_usb = temperature_data_container.clone();

    let out_data = PidData::default();
    let out_data_container = Arc::new(Mutex::new(out_data).expect("Failed to create data guard mutex"));
    let out_data_container_display = out_data_container.clone();
    let out_data_container_pid = out_data_container.clone();
    let out_data_container_main = out_data_container.clone();
    let out_data_container_usb = out_data_container.clone();

    let interface = Interface::default();
    let interface_data_container = Arc::new(Mutex::new(interface).expect("Failed to create data guard mutex"));
    let interface_data_container_display = interface_data_container.clone();
    let interface_data_container_pid = interface_data_container.clone();
    let interface_data_container_main = interface_data_container.clone();
    let interface_data_container_usb = interface_data_container.clone();

    let pump = PumpData::default();
    let pump_data_container = Arc::new(Mutex::new(pump).expect("Failed to create data guard mutex"));
    let pump_data_container_display = pump_data_container.clone();
    let pump_data_container_main = pump_data_container.clone();
    let pump_data_container_usb = pump_data_container.clone();

    delay.delay(1000.millis());
    usb_println("boot up ok");
    fault_1_led.on();
    fault_2_led.on();
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

            let max31865_1_state = max31865_1.init(&mut spi);
            let max31865_2_state = max31865_2.init(&mut spi);
            let max31865_3_state = max31865_3.init(&mut spi);
            let max31865_4_state = max31865_4.init(&mut spi);
            let max31865_5_state = max31865_5.init(&mut spi);

            loop {
                let t1;
                let t2;
                let t3;
                let t4;
                let t5;

                if max31865_1_state.is_ok() {
                    t1 = max31865_1.get_temperature(&mut spi);
                } else {
                    t1 = None;
                }
                if max31865_2_state.is_ok() {
                    t2 = max31865_2.get_temperature(&mut spi);
                } else {
                    t2 = None;
                }
                if max31865_3_state.is_ok() {
                    t3 = max31865_3.get_temperature(&mut spi);
                } else {
                    t3 = None;
                }
                if max31865_4_state.is_ok() {
                    t4 = max31865_4.get_temperature(&mut spi);
                } else {
                    t4 = None;
                }
                if max31865_5_state.is_ok() {
                    t5 = max31865_5.get_temperature(&mut spi);
                } else {
                    t5 = None;
                }

                match temperature_data_container_adc.lock(Duration::ms(1)) {
                    Ok(mut temperature_data) => {
                        temperature_data.t1 = t1;
                        temperature_data.t2 = t2;
                        temperature_data.t3 = t3;
                        temperature_data.t4 = t4;
                        temperature_data.t5 = t5;
                    }
                    Err(_) => {}
                }
                freertos_rust::CurrentTask::delay(Duration::ms(100));
            }
        }).unwrap();

    Task::new()
        .name("PID TASK")
        .stack_size(512)
        .priority(TaskPriority(3))
        .start(move || {
            let mut pid = PID::new();
            loop {
                let mut current_temperature = None;
                match temperature_data_container_pid.lock(Duration::ms(1)) {
                    Ok(temperature_data) => {
                        current_temperature = temperature_data.t5;
                    }
                    Err(_) => {}
                }
                match interface_data_container_pid.lock(Duration::ms(1)) {
                    Ok(interface) => {
                        pid.target = interface.coffee_temperature;
                    }
                    Err(_) => {}
                }
                match out_data_container_pid.lock(Duration::ms(1)) {
                    Ok(mut out_data_temp) => {
                        // get values
                        pid.kp = out_data_temp.kp;
                        pid.ki = out_data_temp.ki;
                        pid.kd = out_data_temp.kd;
                        pid.window_size = out_data_temp.window_size;
                        pid.max_val = out_data_temp.max_val;
                        // push values
                        out_data_temp.p = pid.p;
                        out_data_temp.i = pid.i;
                        out_data_temp.d = pid.d;
                        out_data_temp.pid_val = pid.val;
                        out_data_temp.duty_cycle = pid.duty_cycle;
                    }
                    Err(_) => {}
                }
                match current_temperature {
                    None => {}
                    Some(t) => {
                        // we use a window and set the pin high for the calculated duty_cycle,
                        // this is not really PWM, since the solid state relay only switches at zero-crossing
                        // so we cannot use high frequency pwm
                        // since the heating process is slow, it is okay to have a larger window size
                        let duty_cycle = pid.get_heat_value(t, tick_timer.now().ticks());
                        let now = tick_timer.now().ticks();
                        heater_1.set_high();
                        while now + duty_cycle > tick_timer.now().ticks() {
                            // heater switches at zero crossing, so we wait for one cycle 1/50Hz = 20ms
                            freertos_rust::CurrentTask::delay(Duration::ms(20));
                        }
                        heater_1.set_low();
                        while now + pid.window_size > tick_timer.now().ticks() {
                            // heater switches at zero crossing, so we wait for one cycle 1/50Hz = 20ms
                            freertos_rust::CurrentTask::delay(Duration::ms(20));
                        }
                    }
                }
                freertos_rust::CurrentTask::delay(Duration::ms(1));
            }
        }).unwrap();

    Task::new()
        .name("DISPLAY TASK")
        .stack_size(1024)
        .priority(TaskPriority(2))
        .start(move || {

            // The image is an RGB565 encoded BMP, so specifying the type as `Image<Bmp<Rgb565>>` will read
            // the pixels correctly
            let im: Image<Bmp<Rgb565>> = Image::new(&bmp, Point::new(32, 0));

            // We use the `color_converted` method here to automatically convert the RGB565 image data into
            // BinaryColor values.
            im.draw(&mut display.color_converted()).unwrap();

            display.flush().unwrap();
            freertos_rust::CurrentTask::delay(Duration::ms(1000));

            loop {
                display.clear();

                let mut t1 = None;
                let mut t2 = None;
                let mut t3 = None;
                let mut t4 = None;
                let mut t5 = None;

                match temperature_data_container_display.lock(Duration::ms(1)) {
                    Ok(temperature_data) => {
                        t1 = temperature_data.t1;
                        t2 = temperature_data.t2;
                        t3 = temperature_data.t3;
                        t4 = temperature_data.t4;
                        t5 = temperature_data.t5;
                    }
                    Err(_) => {}
                }

                match t1 {
                    None => {
                        Text::new("T1 = - ",
                                  Point::new(0, 10),
                                  MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        ).draw(&mut display).unwrap();
                    }
                    Some(t) => {
                        Text::new(arrform!(128, "T1 = {:.2}", t).as_str(),
                                  Point::new(0, 10),
                                  MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        ).draw(&mut display).unwrap();
                    }
                }

                match t2 {
                    None => {
                        Text::new("T2 = - ",
                                  Point::new(0, 20),
                                  MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        ).draw(&mut display).unwrap();
                    }
                    Some(t) => {
                        Text::new(arrform!(128, "T2 = {:.2}", t).as_str(),
                                  Point::new(0, 20),
                                  MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        ).draw(&mut display).unwrap();
                    }
                }

                match t3 {
                    None => {
                        Text::new("T3 = - ",
                                  Point::new(0, 30),
                                  MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        ).draw(&mut display).unwrap();
                    }
                    Some(t) => {
                        Text::new(arrform!(128, "T3 = {:.2}", t).as_str(),
                                  Point::new(0, 30),
                                  MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        ).draw(&mut display).unwrap();
                    }
                }

                match t4 {
                    None => {
                        Text::new("T4 = - ",
                                  Point::new(0, 40),
                                  MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        ).draw(&mut display).unwrap();
                    }
                    Some(t) => {
                        Text::new(arrform!(128, "T4 = {:.2}", t).as_str(),
                                  Point::new(0, 40),
                                  MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        ).draw(&mut display).unwrap();
                    }
                }

                match t5 {
                    None => {
                        Text::new("T5 = - ",
                                  Point::new(0, 50),
                                  MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        ).draw(&mut display).unwrap();
                    }
                    Some(t) => {
                        Text::new(arrform!(128, "T5 = {:.2}", t).as_str(),
                                  Point::new(0, 50),
                                  MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        ).draw(&mut display).unwrap();
                    }
                }

                cortex_m::interrupt::free(|cs| {
                    pos = G_ENC_STATE
                        .borrow(cs)
                        .get();
                });

                Text::new(arrform!(128, "enc = {:}", pos).as_str(),
                          Point::new(0, 60),
                          MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                ).draw(&mut display).unwrap();

                display.flush().unwrap();
                freertos_rust::CurrentTask::delay(Duration::ms(100));
            }
        }).unwrap();

    Task::new()
        .name("USB TASK")
        .stack_size(1024)
        .priority(TaskPriority(3))
        .start(move || {
            let mut temperature_data = MeasuredData::default();
            let mut out_data = PidData::default();
            let mut interface = Interface::default();
            let mut pump = PumpData::default();
            let mut hk_rate = 500.0;
            let mut hk = true;
            let mut state = State::Idle;

            loop {
                // gather all states
                match temperature_data_container_usb.lock(Duration::ms(1)) {
                    Ok(temperature_data_temp) => {
                        temperature_data = temperature_data_temp.clone();
                    }
                    Err(_) => {}
                }
                match out_data_container_usb.lock(Duration::ms(1)) {
                    Ok(out_data_temp) => {
                        out_data = out_data_temp.clone();
                    }
                    Err(_) => {}
                }
                match interface_data_container_usb.lock(Duration::ms(1)) {
                    Ok(interface_temp) => {
                        interface = interface_temp.clone();
                    }
                    Err(_) => {}
                }
                match pump_data_container_usb.lock(Duration::ms(1)) {
                    Ok(pump_temp) => {
                        pump = pump_temp.clone();
                    }
                    Err(_) => {}
                }
                match state_container_usb.lock(Duration::ms(1)) {
                    Ok(state_temp) => {
                        state = state_temp.clone();
                    }
                    Err(_) => {}
                }

                send_housekeeping(&state, &temperature_data, &interface, &out_data, "");

                let mut message_bytes = [0; 1024];
                usb_read(&mut message_bytes);
                match core::str::from_utf8(&message_bytes) {
                    Ok(cmd) => {
                        extract_command(cmd, &mut interface, &mut out_data, &mut pump, &mut hk, &mut hk_rate);
                    }
                    Err(_) => {}
                }

                // update containers
                match out_data_container_usb.lock(Duration::ms(1)) {
                    Ok(mut out_data_temp) => {
                        *out_data_temp = out_data.clone();
                    }
                    Err(_) => {}
                }
                match interface_data_container_usb.lock(Duration::ms(1)) {
                    Ok(mut interface_temp) => {
                        *interface_temp = interface.clone();
                    }
                    Err(_) => {}
                }
                match pump_data_container_usb.lock(Duration::ms(1)) {
                    Ok(mut pump_temp) => {
                        *pump_temp = pump.clone();
                    }
                    Err(_) => {}
                }


                // sample frequency
                freertos_rust::CurrentTask::delay(Duration::ms(hk_rate as u32 / 2));
                stat_led.toggle();
                freertos_rust::CurrentTask::delay(Duration::ms(hk_rate as u32 / 2));
                stat_led.toggle();
            }
        }).unwrap();

    Task::new()
        .name("MAIN TASK")
        .stack_size(512)
        .priority(TaskPriority(2))
        .start(move || {
            let mut temperature_data = MeasuredData::default();
            let mut out_data = PidData::default();
            let mut pump = PumpData::default();
            let mut interface = Interface::default();
            let mut led_state = LedState::Off;
            let mut state = State::Idle;
            let max_duty = bldc_pwm.get_max_duty();
            let mut timer = 0;

            loop {

                // gather all containers
                match temperature_data_container_main.lock(Duration::ms(1)) {
                    Ok(temperature_data_temp) => {
                        temperature_data = temperature_data_temp.clone();
                    }
                    Err(_) => {}
                }
                match out_data_container_main.lock(Duration::ms(1)) {
                    Ok(out_data_temp) => {
                        out_data = out_data_temp.clone();
                    }
                    Err(_) => {}
                }
                match interface_data_container_main.lock(Duration::ms(1)) {
                    Ok(interface_temp) => {
                        interface = interface_temp.clone();
                    }
                    Err(_) => {}
                }
                match pump_data_container_main.lock(Duration::ms(1)) {
                    Ok(pump_temp) => {
                        pump = pump_temp.clone();
                    }
                    Err(_) => {}
                }
                match state_container_main.lock(Duration::ms(1)) {
                    Ok(state_temp) => {
                        state = state_temp.clone();
                    }
                    Err(_) => {}
                }

                // state-machine
                match state {
                    State::Idle => {
                        bldc_pwm.set_duty(0);
                        bldc_en.set_high();
                        led_state = LedState::Off;
                        out_data.enable = false;
                        if interface.button && !water_low {  // TODO: check button pin
                            state = State::CoffeeHeating;
                            interface.button = false;
                        }
                    }
                    State::CoffeeHeating => {
                        bldc_pwm.set_duty(max_duty * (pump.heat_up_power / 100.0) as u16);
                        bldc_en.set_low();
                        out_data.enable = true;
                        led_state = LedState::SlowSine;
                        if let Some(temperature) = temperature_data.t1 {
                            if interface.coffee_temperature * 0.95 <= temperature && temperature <= 1.05 * interface.coffee_temperature {
                                state = State::Ready;
                            }
                        }
                    }
                    State::Ready => {
                        bldc_pwm.set_duty(0);
                        bldc_en.set_high();
                        led_state = LedState::On;
                        if interface.lever_switch {  // TODO: check switch pin
                            state = State::PreInfuse;
                            interface.lever_switch = false;
                            timer = 0;
                        }
                    }
                    State::PreInfuse => {
                        bldc_pwm.set_duty(max_duty * (pump.pre_infuse_power / 100.0) as u16);
                        bldc_en.set_low();
                        led_state = LedState::SlowBlink;
                        // timer of 5s
                        if timer >= 100 {
                            state = State::Extracting;
                            timer = 0;
                        }
                    }
                    State::Extracting => {
                        bldc_pwm.set_duty(max_duty * (pump.extract_power / 100.0) as u16);
                        bldc_en.set_low();
                        led_state = LedState::FastBlink;
                        // timeout of 30s
                        if timer >= 3000 {
                            state = State::Ready;
                        }
                    }
                    State::SteamHeating => {
                        bldc_pwm.set_duty(0);
                        bldc_en.set_high();
                        led_state = LedState::SlowSine;
                        if let Some(temperature) = temperature_data.t2 {
                            if interface.steam_temperature * 0.95 <= temperature && temperature <= 1.05 * interface.steam_temperature {
                                state = State::Ready;
                            }
                        }
                    }
                    State::Steaming => {
                        bldc_pwm.set_duty(max_duty * (pump.steam_power / 100.0) as u16);
                        bldc_en.set_low();
                        led_state = LedState::FastBlink;
                    }
                }

                // update timer
                timer += 1;
                if timer >= 10000 {
                    timer = 1;
                }

                // send states
                match led_state_container_main.lock(Duration::ms(1)) {
                    Ok(mut led_state_temp) => {
                        *led_state_temp = led_state.clone();
                    }
                    Err(_) => {}
                }
                match state_container_main.lock(Duration::ms(1)) {
                    Ok(mut state_temp) => {
                        *state_temp = state.clone();
                    }
                    Err(_) => {}
                }
                match interface_data_container_main.lock(Duration::ms(1)) {
                    Ok(mut interface_temp) => {
                        interface_temp.button = interface.button;
                        interface_temp.lever_switch = interface.lever_switch;
                    }
                    Err(_) => {}
                }
                match out_data_container_main.lock(Duration::ms(1)) {
                    Ok(mut out_data_temp) => {
                        out_data_temp.enable = out_data.enable;
                    }
                    Err(_) => {}
                }

                freertos_rust::CurrentTask::delay(Duration::ms(50));
            }
        }).unwrap();

    Task::new()
        .name("LED TASK")
        .stack_size(256)
        .priority(TaskPriority(0))
        .start(move || {
            let mut led_state = LedState::Off;
            let max_duty = led_pwm.get_max_duty();
            let mut count = 0;
            loop {
                match led_state_container_led.lock(Duration::ms(1)) {
                    Ok(led_state_temp) => {
                        led_state = led_state_temp.clone();
                    }
                    Err(_) => {}
                }
                match led_state {
                    LedState::Off => {
                        led_pwm.set_duty(0);
                    }
                    LedState::On => {
                        led_pwm.set_duty(max_duty);
                    }
                    LedState::SlowSine => {
                        let val = max_duty - (max_duty as f32 * (count as f32 / 1024.0 * PI).sin()) as u16; // LED1
                        led_pwm.set_duty(val);
                    }
                    LedState::FastBlink => {
                        led_pwm.set_duty(0);
                        freertos_rust::CurrentTask::delay(Duration::ms(250));
                        led_pwm.set_duty(max_duty);
                        freertos_rust::CurrentTask::delay(Duration::ms(225));
                    }
                    LedState::SlowBlink => {
                        led_pwm.set_duty(0);
                        freertos_rust::CurrentTask::delay(Duration::ms(500));
                        led_pwm.set_duty(max_duty);
                        freertos_rust::CurrentTask::delay(Duration::ms(475));
                    }
                }
                count += 10;
                freertos_rust::CurrentTask::delay(Duration::ms(25));
            }
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
#[allow(non_snake_case)]
unsafe fn DefaultHandler(_irqn: i16) {
// custom default handler
// irqn is negative for Cortex-M exceptions
// irqn is positive for device specific (line IRQ)
// panic!("Exception: {}", irqn);
}

#[exception]
#[allow(non_snake_case)]
unsafe fn HardFault(_ef: &ExceptionFrame) -> ! {
    loop {}
}

// define what happens in an Out Of Memory (OOM) condition
#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    asm::bkpt();
    loop {}
}

#[no_mangle]
#[allow(non_snake_case, unused_variables)]
fn vApplicationStackOverflowHook(pxTask: FreeRtosTaskHandle, pcTaskName: FreeRtosCharPtr) {
    asm::bkpt();
}