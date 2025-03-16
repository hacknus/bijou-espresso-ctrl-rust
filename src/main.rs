//#![deny(unsafe_code)]
#![no_main]
#![no_std]
// For allocator
#![feature(lang_items)]
#![feature(alloc_error_handler)]

extern crate alloc;

use alloc::sync::Arc;
use core::f32::consts::PI;
use core::panic::PanicInfo;

use arrform::{arrform, ArrForm};
use cortex_m::asm;
use cortex_m_rt::exception;
use cortex_m_rt::{entry, ExceptionFrame};
use embedded_graphics::{
    image::Image,
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use freertos_rust::*;
use micromath::F32Ext;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use stm32f4xx_hal::adc::config::{AdcConfig, Dma, SampleTime, Scan, Sequence};
use stm32f4xx_hal::adc::{Adc, Temperature};
use stm32f4xx_hal::dma::config::DmaConfig;
use stm32f4xx_hal::dma::{StreamsTuple, Transfer};
use stm32f4xx_hal::timer::{Channel1, Channel2};
use stm32f4xx_hal::{
    gpio::Edge,
    i2c::Mode as i2cMode,
    otg_fs::USB,
    pac::Peripherals,
    pac::{self, Interrupt},
    prelude::*,
    spi::{Mode as spiMode, Phase, Polarity},
    timer::{Channel, Channel3, Channel4},
};
use tinybmp::Bmp;

use crate::commands::{
    extract_command, send_housekeeping, HeaterCommand, PumpCommand, ValveCommand,
};
use crate::devices::led::LED;
use crate::devices::max31865::Wires;
use crate::devices::max31865::MAX31865;
use crate::devices::pressure_sense::{read_pressure, ADC_MEMORY, G_XFR};
use crate::intrpt::{G_ENC_PIN_A, G_ENC_PIN_B, G_ENC_STATE};
use crate::mav::Mav;
use crate::pid::{PID, STEADY_STATE_BOUNDS};
use crate::usb::{usb_init, usb_println, usb_read};
use crate::utils::{
    CoffeeState, HeaterState, Interface, LedState, MeasuredData, PidData, PumpData, PumpState,
    State, ValveState,
};

mod commands;
mod devices;
mod intrpt;
mod mav;
mod pid;
mod usb;
mod utils;

static mut SHUTDOWN: bool = false;

#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;

fn check_shutdown() -> bool {
    cortex_m::interrupt::free(|_| unsafe { SHUTDOWN })
}

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

    // TODO check if TIM4 actually works, before it was TIM5
    let mut tick_timer = dp.TIM4.counter_ms(&clocks);
    tick_timer.start(2_678_400_000.millis()).unwrap(); // set the timeout to 31 days

    let mut delay = dp.TIM1.delay_us(&clocks);
    delay.delay(100.millis()); // apparently required for USB to set up properly...

    // initialize ports
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();
    let gpioe = dp.GPIOE.split();

    // initialize DMA
    let dma2 = StreamsTuple::new(dp.DMA2);

    // initialize pins
    let cs_5 = gpiod.pd14.into_push_pull_output();
    let cs_4 = gpiod.pd10.into_push_pull_output();
    let cs_3 = gpiod.pd11.into_push_pull_output();
    let cs_2 = gpiod.pd12.into_push_pull_output();
    let cs_1 = gpiod.pd13.into_push_pull_output();
    let mut bldc_en = gpioa.pa8.into_push_pull_output();
    let mut bldc_dir = gpioc.pc9.into_push_pull_output();
    bldc_dir.set_low();
    let water_low = false; // TODO: implement water_low pin
    let heater_1 = Channel1::new(gpioc.pc6);
    let heater_2 = Channel2::new(gpioc.pc7);
    let heater_bg = Channel3::new(gpioa.pa2);

    let lever = gpioe.pe7.into_floating_input();
    let bldc_v = Channel3::new(gpioc.pc8);
    let buzz = Channel4::new(gpioa.pa3);
    let led_dim = Channel4::new(gpiob.pb1);

    let enc_pin_a = gpioe.pe11.into_floating_input();
    let mut enc_pin_b = gpioe.pe9.into_floating_input();
    let _enc_pin_sw = gpioe.pe10.into_floating_input();
    let mut syscfg = dp.SYSCFG.constrain();
    enc_pin_b.make_interrupt_source(&mut syscfg);
    enc_pin_b.trigger_on_edge(&mut dp.EXTI, Edge::RisingFalling);
    enc_pin_b.enable_interrupt(&mut dp.EXTI);

    // initialize leds
    let mut stat_led = LED::new(gpioe.pe2.into_push_pull_output());
    let mut fault_1_led = LED::new(gpioe.pe13.into_push_pull_output());
    let mut fault_2_led = LED::new(gpioe.pe14.into_push_pull_output());

    // initialize button
    let button = gpiob.pb0.into_floating_input();

    // initialize extension pins
    let _pin_a_15 = gpioa.pa15.into_push_pull_output();
    let mut valve1_pin = gpioc.pc10.into_push_pull_output();
    let mut valve2_pin = gpioc.pc11.into_push_pull_output();
    let _pin_c_12 = gpioc.pc12.into_push_pull_output();

    valve1_pin.set_high();
    valve2_pin.set_high();

    // initialize buzzer
    let mut buzz_pwm = dp.TIM2.pwm_hz(buzz, 2000.Hz(), &clocks);
    let max_duty = buzz_pwm.get_max_duty();
    buzz_pwm.set_duty(Channel::C4, max_duty / 2);

    // initialize pwm timer 3
    let (mut bldc_pwm, mut led_pwm) = dp
        .TIM3
        .pwm_hz((bldc_v, led_dim), 10000.Hz(), &clocks)
        .split();

    // initialize dimming LED
    let max_duty = led_pwm.get_max_duty();
    led_pwm.set_duty(max_duty / 2);

    // initialize bldc pwm
    let max_duty = bldc_pwm.get_max_duty();
    bldc_pwm.set_duty(max_duty / 2);

    // initialize heater 1 pwm

    // initialize heater 2 pwm
    let (mut heater_1_pwm, mut heater_2_pwm) = dp
        .TIM8
        .pwm_hz((heater_1, heater_2), 3.Hz(), &clocks)
        .split();
    heater_1_pwm.set_duty(0);
    heater_2_pwm.set_duty(0);

    // initialize heater bg pwm
    let mut heater_bg_pwm = dp.TIM5.pwm_hz(heater_bg, 3.Hz(), &clocks);
    heater_bg_pwm.set_duty(Channel::C3, 0);

    // initialize usb
    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: stm32f4xx_hal::gpio::alt::otg_fs::Dm::PA11(gpioa.pa11.into_alternate()),
        pin_dp: stm32f4xx_hal::gpio::alt::otg_fs::Dp::PA12(gpioa.pa12.into_alternate()),
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
    let i2c = dp.I2C1.i2c(
        (scl, sda),
        i2cMode::Standard {
            frequency: 100.kHz(),
        },
        &clocks,
    );

    let interface = I2CDisplayInterface::new(i2c);
    let bmp =
        Bmp::from_slice(include_bytes!("../images/rust.bmp")).expect("Failed to load BMP image");
    // let bmp_inv = Bmp::from_slice(include_bytes!("../images/rust1.bmp")).expect("Failed to load BMP image");

    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let sclk = gpiob.pb13;
    let sdo = gpiob.pb14;
    let sdi = gpiob.pb15;

    // initialize spi
    let spi = dp.SPI2.spi(
        (sclk, sdo, sdi),
        spiMode {
            polarity: Polarity::IdleHigh,
            phase: Phase::CaptureOnSecondTransition,
        },
        10.kHz(),
        &clocks,
    );

    let spi_bus = shared_bus::BusManagerSimple::new(spi);

    // init adc
    let pressure_sense_pin = gpioc.pc0.into_analog();
    let current_sense_pin = gpioc.pc1.into_analog();

    let dma_config = DmaConfig::default()
        .transfer_complete_interrupt(true)
        .memory_increment(true)
        .double_buffer(false);

    let adc_config = AdcConfig::default()
        .dma(Dma::Continuous)
        .scan(Scan::Enabled);

    let mut adc = Adc::adc1(dp.ADC1, true, adc_config);

    adc.configure_channel(&Temperature, Sequence::One, SampleTime::Cycles_480);
    adc.configure_channel(&pressure_sense_pin, Sequence::Two, SampleTime::Cycles_480);
    adc.configure_channel(&current_sense_pin, Sequence::Three, SampleTime::Cycles_480);
    adc.enable_temperature_and_vref();

    // let adc_buffer = cortex_m::singleton!(: [u16; 2] = [0; 2]).unwrap();
    let mut transfer = unsafe {
        Transfer::init_peripheral_to_memory(dma2.0, adc, &mut ADC_MEMORY, None, dma_config)
    };

    transfer.start(|adc| {
        adc.start_conversion();
    });
    cortex_m::interrupt::free(|cs| {
        G_XFR.borrow(cs).replace(Some(transfer));
    });

    let led_state = LedState::Off;
    let led_state_container =
        Arc::new(Mutex::new(led_state).expect("Failed to create data guard mutex"));
    let led_state_container_main = led_state_container.clone();
    let led_state_container_led = led_state_container;

    let state = State::default();
    let state_container = Arc::new(Mutex::new(state).expect("Failed to create data guard mutex"));
    let state_container_main = state_container.clone();
    let state_container_usb = state_container.clone();
    let _state_container_pid = state_container;

    let measured_data = MeasuredData::default();
    let measured_data_container =
        Arc::new(Mutex::new(measured_data).expect("Failed to create data guard mutex"));
    let measured_data_container_display = measured_data_container.clone();
    let measured_data_container_adc = measured_data_container.clone();
    let measured_data_container_main = measured_data_container.clone();
    let measured_data_container_pid_1 = measured_data_container.clone();
    let measured_data_container_pid_2 = measured_data_container.clone();
    let measured_data_container_pid_bg = measured_data_container.clone();
    let measured_data_container_usb = measured_data_container;

    let pid_data_1 = PidData::default();
    let pid_data_1_container =
        Arc::new(Mutex::new(pid_data_1).expect("Failed to create data guard mutex"));
    let _pid_data_1_container_display = pid_data_1_container.clone();
    let pid_1_data_container_pid_1 = pid_data_1_container.clone();
    let pid_1_data_container_main = pid_data_1_container.clone();
    let pid_1_data_container_usb = pid_data_1_container;

    let pid_data_2 = PidData::default();
    let pid_data_2_container =
        Arc::new(Mutex::new(pid_data_2).expect("Failed to create data guard mutex"));
    let _pid_data_2_container_display = pid_data_2_container.clone();
    let pid_2_data_container_pid_2 = pid_data_2_container.clone();
    let pid_2_data_container_main = pid_data_2_container.clone();
    let pid_2_data_container_usb = pid_data_2_container;

    let pid_data_bg = PidData::default();
    let pid_data_bg_container =
        Arc::new(Mutex::new(pid_data_bg).expect("Failed to create data guard mutex"));
    let _pid_data_bg_container_display = pid_data_bg_container.clone();
    let pid_bg_data_container_pid_bg = pid_data_bg_container.clone();
    let pid_bg_data_container_main = pid_data_bg_container.clone();
    let pid_bg_data_container_usb = pid_data_bg_container;

    let interface = Interface::default();
    let interface_data_container =
        Arc::new(Mutex::new(interface).expect("Failed to create data guard mutex"));
    let _interface_data_container_display = interface_data_container.clone();
    let interface_data_container_main = interface_data_container.clone();
    let interface_data_container_usb = interface_data_container;

    let pump = PumpData::default();
    let pump_data_container =
        Arc::new(Mutex::new(pump).expect("Failed to create data guard mutex"));
    let _pump_data_container_display = pump_data_container.clone();
    let pump_data_container_main = pump_data_container.clone();
    let pump_data_container_usb = pump_data_container;

    let pump_command_queue = Arc::new(Queue::new(10).unwrap());
    let pump_command_queue_main = pump_command_queue.clone();
    let pump_command_queue_usb = pump_command_queue;

    let heater_1_command_queue = Arc::new(Queue::new(10).unwrap());
    let heater_1_command_queue_pid_1 = heater_1_command_queue.clone();
    let heater_1_command_queue_usb = heater_1_command_queue;

    let heater_2_command_queue = Arc::new(Queue::new(10).unwrap());
    let heater_2_command_queue_pid_2 = heater_2_command_queue.clone();
    let heater_2_command_queue_usb = heater_2_command_queue;

    let heater_bg_command_queue = Arc::new(Queue::new(10).unwrap());
    let heater_bg_command_queue_pid_bg = heater_bg_command_queue.clone();
    let heater_bg_command_queue_usb = heater_bg_command_queue;

    let valve_command_queue = Arc::new(Queue::new(10).unwrap());
    let valve_command_queue_main = valve_command_queue.clone();
    let valve_command_queue_usb = valve_command_queue;

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
            let mut max31865_1 = MAX31865::new(spi_bus.acquire_spi(), cs_1, Wires::TwoWire);
            let mut max31865_2 = MAX31865::new(spi_bus.acquire_spi(), cs_2, Wires::TwoWire);
            let mut max31865_3 = MAX31865::new(spi_bus.acquire_spi(), cs_3, Wires::TwoWire);
            let mut max31865_4 = MAX31865::new(spi_bus.acquire_spi(), cs_4, Wires::TwoWire);
            let mut max31865_5 = MAX31865::new(spi_bus.acquire_spi(), cs_5, Wires::TwoWire);

            let max31865_1_state = max31865_1.init();
            let max31865_2_state = max31865_2.init();
            let max31865_3_state = max31865_3.init();
            let max31865_4_state = max31865_4.init();
            let max31865_5_state = max31865_5.init();

            loop {
                if check_shutdown() {
                    break;
                }

                let pressure = read_pressure();

                let t1 = if max31865_1_state.is_ok() {
                    max31865_1.get_temperature()
                } else {
                    None
                };
                let t2 = if max31865_2_state.is_ok() {
                    max31865_2.get_temperature()
                } else {
                    None
                };
                let t3 = if max31865_3_state.is_ok() {
                    max31865_3.get_temperature()
                } else {
                    None
                };
                let t4 = if max31865_4_state.is_ok() {
                    max31865_4.get_temperature()
                } else {
                    None
                };
                let t5 = if max31865_5_state.is_ok() {
                    max31865_5.get_temperature()
                } else {
                    None
                };

                if let Ok(mut measured_data) = measured_data_container_adc.lock(Duration::ms(5)) {
                    measured_data.t1 = t1;
                    measured_data.t2 = t2;
                    measured_data.t3 = t3;
                    measured_data.t4 = t4;
                    measured_data.t5 = t5;
                    measured_data.p = pressure;
                }
                CurrentTask::delay(Duration::ms(100));
            }
        })
        .unwrap();

    Task::new()
        .name("PID TASK")
        .stack_size(512)
        .priority(TaskPriority(1))
        .start(move || {
            let mut heater_1_pid = PID::new();
            let mut heater_1_current_temperature = None;
            let mut heater_1_boiler_override = None;
            let mut heater_2_pid = PID::new();
            let mut heater_2_current_temperature = None;
            let mut heater_2_boiler_override = None;
            let mut heater_bg_pid = PID::new();
            let mut heater_bg_current_temperature = None;
            let mut heater_bg_boiler_override = None;

            let mut state = State::default();

            let mut mav_1 = Mav::new();
            let mut mav_2 = Mav::new();
            let mut mav_bg = Mav::new();

            let mut mav_1_counter = 0;
            let mut mav_2_counter = 0;
            let mut mav_bg_counter = 0;
            let mav_update_interval = 10;

            loop {
                if check_shutdown() {
                    heater_1_pwm.disable();
                    heater_1_pwm.set_duty(0);
                    heater_2_pwm.disable();
                    heater_2_pwm.set_duty(0);
                    heater_bg_pwm.disable(Channel::C3);
                    heater_bg_pwm.set_duty(Channel::C3, 0);
                    fault_2_led.off();
                    break;
                }

                mav_1_counter += 1;
                mav_2_counter += 1;
                mav_bg_counter += 1;

                if let Ok(cmd) = heater_1_command_queue_pid_1.receive(Duration::ms(5)) {
                    match cmd {
                        HeaterCommand::Temperature(temperature) => {
                            heater_1_pid.target = temperature
                        }
                        HeaterCommand::Heating(enable) => heater_1_pid.enabled = enable,
                        HeaterCommand::WindowSize(window_size) => {
                            heater_1_pid.window_size = window_size as u32;
                        }
                        HeaterCommand::PidP(kp) => heater_1_pid.kp = kp,
                        HeaterCommand::PidI(ki) => heater_1_pid.ki = ki,
                        HeaterCommand::PidD(kd) => heater_1_pid.kd = kd,
                        HeaterCommand::PidMaxVal(max_val) => heater_1_pid.max_val = max_val,
                        HeaterCommand::Boiler1(boiler_1) => heater_1_boiler_override = boiler_1,
                    }
                }

                if let Ok(mut pid_temp) = pid_1_data_container_pid_1.lock(Duration::ms(10)) {
                    // get values
                    heater_1_pid.enabled = pid_temp.enable;
                    heater_1_pid.kp = pid_temp.kp;
                    heater_1_pid.ki = pid_temp.ki;
                    heater_1_pid.kd = pid_temp.kd;
                    heater_1_pid.window_size = pid_temp.window_size;
                    heater_1_pid.max_val = pid_temp.max_val;
                    heater_1_pid.target = pid_temp.target;
                    // update current temperature for state machine
                    if let Ok(data) = measured_data_container_pid_1.lock(Duration::ms(5)) {
                        heater_1_current_temperature = data.t2;
                    }
                    pid_temp.current_temperature = heater_1_current_temperature;
                    // check if i has been reset
                    if pid_temp.reset_i {
                        heater_1_pid.i = 0.0;
                        pid_temp.reset_i = false;
                    }
                    // push values
                    pid_temp.p = heater_1_pid.p;
                    pid_temp.i = heater_1_pid.i;
                    pid_temp.d = heater_1_pid.d;
                    pid_temp.pid_val = heater_1_pid.val;
                    pid_temp.duty_cycle = heater_1_pid.duty_cycle;

                    // state-machine
                    match state.heater_1_state {
                        HeaterState::Off => {
                            if pid_temp.enable {
                                if let Some(temperature) = pid_temp.current_temperature {
                                    if heater_1_pid.target < temperature {
                                        state.heater_1_state = HeaterState::CoolDown;
                                    } else {
                                        state.heater_1_state = HeaterState::HeatUp;
                                    }
                                }
                            };
                        }
                        HeaterState::HeatUp | HeaterState::CoolDown => {
                            if let Some(temperature) = pid_temp.current_temperature {
                                if heater_1_pid.target < temperature {
                                    state.heater_1_state = HeaterState::CoolDown;
                                } else {
                                    state.heater_1_state = HeaterState::HeatUp;
                                }
                                // go in steady state if we are inside +/- steady state bounds K of target
                                if heater_1_pid.target - STEADY_STATE_BOUNDS <= mav_1.evaluate()
                                    && mav_1.evaluate() <= STEADY_STATE_BOUNDS + heater_1_pid.target
                                    && heater_1_pid.target - STEADY_STATE_BOUNDS <= temperature
                                    && temperature <= STEADY_STATE_BOUNDS + heater_1_pid.target
                                    && mav_1.spread() < STEADY_STATE_BOUNDS / 2.0
                                {
                                    state.heater_1_state = HeaterState::SteadyState;
                                }
                            }
                            if !pid_temp.enable {
                                state.heater_1_state = HeaterState::Off;
                            }
                        }
                        HeaterState::SteadyState => {
                            if let Some(temperature) = pid_temp.current_temperature {
                                // if temperature falls outside of +/- steady state bounds K around target
                                // then we are not in steady state
                                if (heater_1_pid.target - STEADY_STATE_BOUNDS > mav_1.evaluate()
                                    || mav_1.evaluate() > STEADY_STATE_BOUNDS + heater_1_pid.target)
                                    || (heater_1_pid.target - STEADY_STATE_BOUNDS > temperature
                                        || temperature > STEADY_STATE_BOUNDS + heater_1_pid.target)
                                {
                                    if heater_1_pid.target < temperature {
                                        state.heater_1_state = HeaterState::CoolDown;
                                    } else {
                                        state.heater_1_state = HeaterState::HeatUp;
                                    }
                                }
                            }
                            if !pid_temp.enable {
                                state.heater_1_state = HeaterState::Off;
                            }
                        }
                    }
                }
                match heater_1_current_temperature {
                    None => {
                        // if we have no temperature, we need to turn off the heater
                        heater_1_pwm.disable();
                        heater_1_pwm.set_duty(0);
                    }
                    Some(t) => {
                        if mav_1_counter > mav_update_interval {
                            mav_1_counter = 0;
                            mav_1.push(t);
                        }
                        // we use a window and set the pin high for the calculated duty_cycle,
                        // this is not really PWM, since the solid state relay only switches at zero-crossing
                        // so we cannot use high frequency pwm
                        // since the heating process is slow, it is okay to have a larger window size
                        let duty_cycle = heater_1_pid.get_heat_value(t, tick_timer.now().ticks());
                        let max_duty = heater_1_pwm.get_max_duty();
                        if heater_1_pid.enabled {
                            heater_1_pwm.enable();
                            heater_1_pwm
                                .set_duty((duty_cycle.clamp(0.0, 1.0) * (max_duty as f32)) as u16);
                        } else {
                            heater_1_pwm.disable();
                            heater_1_pwm.set_duty(0);
                        }
                    }
                }

                if let Ok(cmd) = heater_2_command_queue_pid_2.receive(Duration::ms(5)) {
                    match cmd {
                        HeaterCommand::Temperature(temperature) => {
                            heater_2_pid.target = temperature
                        }
                        HeaterCommand::Heating(enable) => heater_2_pid.enabled = enable,
                        HeaterCommand::WindowSize(window_size) => {
                            heater_2_pid.window_size = window_size as u32;
                        }
                        HeaterCommand::PidP(kp) => heater_2_pid.kp = kp,
                        HeaterCommand::PidI(ki) => heater_2_pid.ki = ki,
                        HeaterCommand::PidD(kd) => heater_2_pid.kd = kd,
                        HeaterCommand::PidMaxVal(max_val) => heater_2_pid.max_val = max_val,
                        HeaterCommand::Boiler1(boiler_1) => heater_2_boiler_override = boiler_1,
                    }
                }

                if let Ok(mut pid_temp) = pid_2_data_container_pid_2.lock(Duration::ms(10)) {
                    // get values
                    heater_2_pid.enabled = pid_temp.enable;
                    heater_2_pid.kp = pid_temp.kp;
                    heater_2_pid.ki = pid_temp.ki;
                    heater_2_pid.kd = pid_temp.kd;
                    heater_2_pid.window_size = pid_temp.window_size;
                    heater_2_pid.max_val = pid_temp.max_val;
                    heater_2_pid.target = pid_temp.target;
                    // update current temperature for state machine
                    if let Ok(data) = measured_data_container_pid_2.lock(Duration::ms(5)) {
                        // TODO
                        heater_2_current_temperature = data.t1;
                    }
                    pid_temp.current_temperature = heater_2_current_temperature;
                    // check if i has been reset
                    if pid_temp.reset_i {
                        heater_2_pid.i = 0.0;
                        pid_temp.reset_i = false;
                    }
                    // push values
                    pid_temp.p = heater_2_pid.p;
                    pid_temp.i = heater_2_pid.i;
                    pid_temp.d = heater_2_pid.d;
                    pid_temp.pid_val = heater_2_pid.val;
                    pid_temp.duty_cycle = heater_2_pid.duty_cycle;

                    // state-machine
                    match state.heater_2_state {
                        HeaterState::Off => {
                            if pid_temp.enable {
                                if let Some(temperature) = pid_temp.current_temperature {
                                    if heater_2_pid.target < temperature {
                                        state.heater_2_state = HeaterState::CoolDown;
                                    } else {
                                        state.heater_2_state = HeaterState::HeatUp;
                                    }
                                }
                            };
                        }
                        HeaterState::HeatUp | HeaterState::CoolDown => {
                            if let Some(temperature) = pid_temp.current_temperature {
                                if heater_2_pid.target < temperature {
                                    state.heater_2_state = HeaterState::CoolDown;
                                } else {
                                    state.heater_2_state = HeaterState::HeatUp;
                                }
                                // go in steady state if we are inside +/- steady state bounds K of target
                                if heater_2_pid.target - STEADY_STATE_BOUNDS <= mav_2.evaluate()
                                    && mav_2.evaluate() <= STEADY_STATE_BOUNDS + heater_2_pid.target
                                    && heater_2_pid.target - STEADY_STATE_BOUNDS <= temperature
                                    && temperature <= STEADY_STATE_BOUNDS + heater_2_pid.target
                                    && mav_2.spread() < STEADY_STATE_BOUNDS / 2.0
                                {
                                    state.heater_2_state = HeaterState::SteadyState;
                                }
                            }
                            if !pid_temp.enable {
                                state.heater_2_state = HeaterState::Off;
                            }
                        }
                        HeaterState::SteadyState => {
                            if let Some(temperature) = pid_temp.current_temperature {
                                // if temperature falls outside of +/- steady state bounds K around target
                                // then we are not in steady state
                                if (heater_2_pid.target - STEADY_STATE_BOUNDS > mav_2.evaluate()
                                    || mav_2.evaluate() > STEADY_STATE_BOUNDS + heater_2_pid.target)
                                    || (heater_2_pid.target - STEADY_STATE_BOUNDS > temperature
                                        || temperature > STEADY_STATE_BOUNDS + heater_2_pid.target)
                                {
                                    if heater_2_pid.target < temperature {
                                        state.heater_2_state = HeaterState::CoolDown;
                                    } else {
                                        state.heater_2_state = HeaterState::HeatUp;
                                    }
                                }
                            }
                            if !pid_temp.enable {
                                state.heater_2_state = HeaterState::Off;
                            }
                        }
                    }
                }
                match heater_2_current_temperature {
                    None => {
                        // if we have no temperature, we need to turn off the heater
                        heater_2_pwm.disable();
                        heater_2_pwm.set_duty(0);
                    }
                    Some(t) => {
                        if mav_2_counter > mav_update_interval {
                            mav_2_counter = 0;
                            mav_2.push(t);
                        }
                        // we use a window and set the pin high for the calculated duty_cycle,
                        // this is not really PWM, since the solid state relay only switches at zero-crossing
                        // so we cannot use high frequency pwm
                        // since the heating process is slow, it is okay to have a larger window size
                        let duty_cycle = heater_2_pid.get_heat_value(t, tick_timer.now().ticks());
                        let max_duty = heater_2_pwm.get_max_duty();
                        if heater_2_pid.enabled {
                            heater_2_pwm.enable();
                            heater_2_pwm
                                .set_duty((duty_cycle.clamp(0.0, 1.0) * (max_duty as f32)) as u16);
                        } else {
                            heater_2_pwm.disable();
                            heater_2_pwm.set_duty(0);
                        }
                    }
                }

                if let Ok(cmd) = heater_bg_command_queue_pid_bg.receive(Duration::ms(5)) {
                    match cmd {
                        HeaterCommand::Temperature(temperature) => {
                            heater_bg_pid.target = temperature
                        }
                        HeaterCommand::Heating(enable) => heater_bg_pid.enabled = enable,
                        HeaterCommand::WindowSize(window_size) => {
                            heater_bg_pid.window_size = window_size as u32;
                        }
                        HeaterCommand::PidP(kp) => heater_bg_pid.kp = kp,
                        HeaterCommand::PidI(ki) => heater_bg_pid.ki = ki,
                        HeaterCommand::PidD(kd) => heater_bg_pid.kd = kd,
                        HeaterCommand::PidMaxVal(max_val) => heater_bg_pid.max_val = max_val,
                        HeaterCommand::Boiler1(boiler_1) => heater_bg_boiler_override = boiler_1,
                    }
                }

                if let Ok(mut pid_temp) = pid_bg_data_container_pid_bg.lock(Duration::ms(10)) {
                    // get values
                    heater_bg_pid.enabled = pid_temp.enable;
                    heater_bg_pid.kp = pid_temp.kp;
                    heater_bg_pid.ki = pid_temp.ki;
                    heater_bg_pid.kd = pid_temp.kd;
                    heater_bg_pid.window_size = pid_temp.window_size;
                    heater_bg_pid.max_val = pid_temp.max_val;
                    heater_bg_pid.target = pid_temp.target;
                    // update current temperature for state machine
                    if let Ok(data) = measured_data_container_pid_bg.lock(Duration::ms(5)) {
                        heater_bg_current_temperature = data.t3;
                    }
                    pid_temp.current_temperature = heater_bg_current_temperature;
                    // check if i has been reset
                    if pid_temp.reset_i {
                        heater_bg_pid.i = 0.0;
                        pid_temp.reset_i = false;
                    }
                    // push values
                    pid_temp.p = heater_bg_pid.p;
                    pid_temp.i = heater_bg_pid.i;
                    pid_temp.d = heater_bg_pid.d;
                    pid_temp.pid_val = heater_bg_pid.val;
                    pid_temp.duty_cycle = heater_bg_pid.duty_cycle;

                    // state-machine
                    match state.heater_bg_state {
                        HeaterState::Off => {
                            if pid_temp.enable {
                                if let Some(temperature) = pid_temp.current_temperature {
                                    if heater_bg_pid.target < temperature {
                                        state.heater_bg_state = HeaterState::CoolDown;
                                    } else {
                                        state.heater_bg_state = HeaterState::HeatUp;
                                    }
                                }
                            };
                        }
                        HeaterState::HeatUp | HeaterState::CoolDown => {
                            if let Some(temperature) = pid_temp.current_temperature {
                                if heater_bg_pid.target < temperature {
                                    state.heater_bg_state = HeaterState::CoolDown;
                                } else {
                                    state.heater_bg_state = HeaterState::HeatUp;
                                }
                                // go in steady state if we are inside +/- steady state bounds K of target
                                if heater_bg_pid.target - STEADY_STATE_BOUNDS <= mav_bg.evaluate()
                                    && mav_bg.evaluate()
                                        <= STEADY_STATE_BOUNDS + heater_bg_pid.target
                                    && heater_bg_pid.target - STEADY_STATE_BOUNDS <= temperature
                                    && temperature <= STEADY_STATE_BOUNDS + heater_bg_pid.target
                                    && mav_bg.spread() < STEADY_STATE_BOUNDS / 2.0
                                {
                                    state.heater_bg_state = HeaterState::SteadyState;
                                }
                            }
                            if !pid_temp.enable {
                                state.heater_bg_state = HeaterState::Off;
                            }
                        }
                        HeaterState::SteadyState => {
                            if let Some(temperature) = pid_temp.current_temperature {
                                // if temperature falls outside of +/- steady state bounds K around target
                                // then we are not in steady state
                                if (heater_bg_pid.target - STEADY_STATE_BOUNDS > mav_bg.evaluate()
                                    || mav_bg.evaluate()
                                        > STEADY_STATE_BOUNDS + heater_bg_pid.target)
                                    || (heater_bg_pid.target - STEADY_STATE_BOUNDS > temperature
                                        || temperature > STEADY_STATE_BOUNDS + heater_bg_pid.target)
                                {
                                    if heater_bg_pid.target < temperature {
                                        state.heater_bg_state = HeaterState::CoolDown;
                                    } else {
                                        state.heater_bg_state = HeaterState::HeatUp;
                                    }
                                }
                            }
                            if !pid_temp.enable {
                                state.heater_bg_state = HeaterState::Off;
                            }
                        }
                    }
                }
                match heater_bg_current_temperature {
                    None => {
                        // if we have no temperature, we need to turn off the heater
                        heater_bg_pwm.disable(Channel::C3);
                        heater_bg_pwm.set_duty(Channel::C3, 0);
                    }
                    Some(t) => {
                        if mav_bg_counter > mav_update_interval {
                            mav_bg_counter = 0;
                            mav_bg.push(t);
                        }
                        // we use a window and set the pin high for the calculated duty_cycle,
                        // this is not really PWM, since the solid state relay only switches at zero-crossing
                        // so we cannot use high frequency pwm
                        // since the heating process is slow, it is okay to have a larger window size
                        let duty_cycle = heater_bg_pid.get_heat_value(t, tick_timer.now().ticks());
                        let max_duty = heater_bg_pwm.get_max_duty();
                        if heater_bg_pid.enabled {
                            heater_bg_pwm.enable(Channel::C3);
                            heater_bg_pwm.set_duty(
                                Channel::C3,
                                (duty_cycle.clamp(0.0, 1.0) * (max_duty as f32)) as u16,
                            );
                        } else {
                            heater_bg_pwm.disable(Channel::C3);
                            heater_bg_pwm.set_duty(Channel::C3, 0);
                        }
                    }
                }

                CurrentTask::delay(Duration::ms(heater_1_pid.window_size));
            }
        })
        .unwrap();

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
                if check_shutdown() {
                    break;
                }
                display.clear(BinaryColor::Off).unwrap();

                let mut t1 = None;
                let mut t2 = None;
                let mut t3 = None;
                let mut t4 = None;
                let mut t5 = None;

                if let Ok(temperature_data) = measured_data_container_display.lock(Duration::ms(5))
                {
                    t1 = temperature_data.t1;
                    t2 = temperature_data.t2;
                    t3 = temperature_data.t3;
                    t4 = temperature_data.t4;
                    t5 = temperature_data.t5;
                }

                match t1 {
                    None => {
                        Text::new(
                            "T1 = - ",
                            Point::new(0, 10),
                            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        )
                        .draw(&mut display)
                        .unwrap();
                    }
                    Some(t) => {
                        Text::new(
                            arrform!(128, "T1 = {:.2}", t).as_str(),
                            Point::new(0, 10),
                            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        )
                        .draw(&mut display)
                        .unwrap();
                    }
                }

                match t2 {
                    None => {
                        Text::new(
                            "T2 = - ",
                            Point::new(0, 20),
                            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        )
                        .draw(&mut display)
                        .unwrap();
                    }
                    Some(t) => {
                        Text::new(
                            arrform!(128, "T2 = {:.2}", t).as_str(),
                            Point::new(0, 20),
                            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        )
                        .draw(&mut display)
                        .unwrap();
                    }
                }

                match t3 {
                    None => {
                        Text::new(
                            "T3 = - ",
                            Point::new(0, 30),
                            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        )
                        .draw(&mut display)
                        .unwrap();
                    }
                    Some(t) => {
                        Text::new(
                            arrform!(128, "T3 = {:.2}", t).as_str(),
                            Point::new(0, 30),
                            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        )
                        .draw(&mut display)
                        .unwrap();
                    }
                }

                match t4 {
                    None => {
                        Text::new(
                            "T4 = - ",
                            Point::new(0, 40),
                            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        )
                        .draw(&mut display)
                        .unwrap();
                    }
                    Some(t) => {
                        Text::new(
                            arrform!(128, "T4 = {:.2}", t).as_str(),
                            Point::new(0, 40),
                            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        )
                        .draw(&mut display)
                        .unwrap();
                    }
                }

                match t5 {
                    None => {
                        Text::new(
                            "T5 = - ",
                            Point::new(0, 50),
                            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        )
                        .draw(&mut display)
                        .unwrap();
                    }
                    Some(t) => {
                        Text::new(
                            arrform!(128, "T5 = {:.2}", t).as_str(),
                            Point::new(0, 50),
                            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                        )
                        .draw(&mut display)
                        .unwrap();
                    }
                }

                cortex_m::interrupt::free(|cs| {
                    pos = G_ENC_STATE.borrow(cs).get();
                });

                Text::new(
                    arrform!(128, "enc = {:}", pos).as_str(),
                    Point::new(0, 60),
                    MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                )
                .draw(&mut display)
                .unwrap();

                display.flush().unwrap();
                freertos_rust::CurrentTask::delay(Duration::ms(100));
            }
        })
        .unwrap();

    Task::new()
        .name("USB TASK")
        .stack_size(1024)
        .priority(TaskPriority(3))
        .start(move || {
            let mut temperature_data = MeasuredData::default();
            let mut pid_1_data = PidData::default();
            let mut pid_2_data = PidData::default();
            let mut pid_bg_data = PidData::default();
            let mut interface = Interface::default();
            let mut pump = PumpData::default();
            let mut hk_rate = 500.0;
            let mut hk = true;
            let mut state = State::default();

            loop {
                if check_shutdown() {
                    usb_println("a panic occurred... stopping all threads...");
                    CurrentTask::delay(Duration::ms(1000));
                    break;
                }

                // gather all states
                if let Ok(temperature_data_temp) = measured_data_container_usb.lock(Duration::ms(5))
                {
                    temperature_data = temperature_data_temp.clone();
                }
                if let Ok(pid_data_temp) = pid_1_data_container_usb.lock(Duration::ms(5)) {
                    pid_1_data = pid_data_temp.clone();
                }
                if let Ok(pid_data_temp) = pid_2_data_container_usb.lock(Duration::ms(5)) {
                    pid_2_data = pid_data_temp.clone();
                }
                if let Ok(pid_data_temp) = pid_bg_data_container_usb.lock(Duration::ms(5)) {
                    pid_bg_data = pid_data_temp.clone();
                }
                if let Ok(interface_temp) = interface_data_container_usb.lock(Duration::ms(5)) {
                    interface = interface_temp.clone();
                }
                if let Ok(pump_temp) = pump_data_container_usb.lock(Duration::ms(5)) {
                    pump = pump_temp.clone();
                }
                if let Ok(state_temp) = state_container_usb.lock(Duration::ms(5)) {
                    state = state_temp.clone();
                }

                send_housekeeping(
                    &state,
                    &temperature_data,
                    &interface,
                    &pid_1_data,
                    &pump,
                    "",
                );

                let mut message_bytes = [0; 1024];
                usb_read(&mut message_bytes);
                if let Ok(cmd) = core::str::from_utf8(&message_bytes) {
                    extract_command(
                        cmd,
                        &heater_1_command_queue_usb,
                        &heater_2_command_queue_usb,
                        &heater_bg_command_queue_usb,
                        &pump_command_queue_usb,
                        &valve_command_queue_usb,
                        &mut hk,
                        &mut hk_rate,
                    );
                }

                // sample frequency
                CurrentTask::delay(Duration::ms(hk_rate as u32 / 2));
                stat_led.toggle();
                CurrentTask::delay(Duration::ms(hk_rate as u32 / 2));
                stat_led.toggle();
            }
        })
        .unwrap();

    Task::new()
        .name("MAIN TASK")
        .stack_size(512)
        .priority(TaskPriority(2))
        .start(move || {
            let mut temperature_data = MeasuredData::default();
            let mut pid_1_data = PidData::default();
            let mut pid_2_data = PidData::default();
            let mut pid_bg_data = PidData::default();
            let mut pump = PumpData::default();
            let mut interface = Interface::default();
            let mut led_state;
            let mut state = State::default();
            let max_duty = bldc_pwm.get_max_duty();
            let mut timer = 0;
            let main_task_period: u32 = 100;
            let mut extraction_time = 20 * main_task_period;

            let mut valve_1_override = None;
            let mut valve_2_override = None;

            let mut pump_override = None;

            let mut previous_kp = pid_1_data.kp;
            let mut previous_ki = pid_1_data.ki;
            let mut previous_kd = pid_1_data.kd;
            let mut previous_target = pid_1_data.target;

            loop {
                if check_shutdown() {
                    break;
                }

                // gather all containers
                if let Ok(temperature_data_temp) =
                    measured_data_container_main.lock(Duration::ms(5))
                {
                    temperature_data = temperature_data_temp.clone();
                }
                if let Ok(pid_data_temp) = pid_1_data_container_main.lock(Duration::ms(5)) {
                    pid_1_data = pid_data_temp.clone();
                }
                if let Ok(pid_data_temp) = pid_2_data_container_main.lock(Duration::ms(5)) {
                    pid_2_data = pid_data_temp.clone();
                }
                if let Ok(pid_data_temp) = pid_bg_data_container_main.lock(Duration::ms(5)) {
                    pid_bg_data = pid_data_temp.clone();
                }
                if let Ok(interface_temp) = interface_data_container_main.lock(Duration::ms(5)) {
                    interface = interface_temp.clone();
                }
                if let Ok(pump_temp) = pump_data_container_main.lock(Duration::ms(5)) {
                    pump = pump_temp.clone();
                }
                if let Ok(state_temp) = state_container_main.lock(Duration::ms(5)) {
                    state = state_temp.clone();
                }

                if let Ok(cmd) = valve_command_queue_main.receive(Duration::ms(5)) {
                    match cmd {
                        ValveCommand::Valve1(state) => {
                            valve_1_override = state;
                        }
                        ValveCommand::Valve2(state) => {
                            valve_2_override = state;
                        }
                    }
                }

                if let Ok(cmd) = pump_command_queue_main.receive(Duration::ms(5)) {
                    match cmd {
                        PumpCommand::PumpOverride(state) => {
                            pump_override = state;
                        }
                        PumpCommand::PumpPower(pwr) => {
                            if let Ok(mut pump_temp) =
                                pump_data_container_main.lock(Duration::ms(5))
                            {
                                pump_temp.extract_power = pwr as f32;
                            }
                        }
                        PumpCommand::PumpHeatUpPower(pwr) => {
                            if let Ok(mut pump_temp) =
                                pump_data_container_main.lock(Duration::ms(5))
                            {
                                pump_temp.heat_up_power = pwr as f32;
                            }
                        }
                        PumpCommand::PumpPreInfusePower(pwr) => {
                            if let Ok(mut pump_temp) =
                                pump_data_container_main.lock(Duration::ms(5))
                            {
                                pump_temp.pre_infuse_power = pwr as f32;
                            }
                        }
                        PumpCommand::PumpCoffeePower(pwr) => {
                            if let Ok(mut pump_temp) =
                                pump_data_container_main.lock(Duration::ms(5))
                            {
                                pump_temp.extract_power = pwr as f32;
                            }
                        }
                        PumpCommand::PumpSteamPower(pwr) => {
                            if let Ok(mut pump_temp) =
                                pump_data_container_main.lock(Duration::ms(5))
                            {
                                pump_temp.steam_power = pwr as f32;
                            }
                        }
                    }
                }

                // Main State-Machine

                interface.button = button.is_low();
                interface.lever_switch = lever.is_low();

                match state.coffee_state {
                    CoffeeState::Idle => {
                        state.pump_state = PumpState::Off;

                        state.valve_1_state = ValveState::Closed;
                        state.valve_2_state = ValveState::Closed;

                        led_state = LedState::Off;
                        pid_1_data.enable = false;
                        pid_2_data.enable = false;
                        pid_bg_data.enable = true;
                        pid_bg_data.target = 60.0;
                        if (interface.button || pid_1_data.enable) && !water_low {
                            state.coffee_state = CoffeeState::CoffeeHeating;
                            state.heater_1_state = HeaterState::HeatUp;
                            state.heater_bg_state = HeaterState::HeatUp;
                        }
                    }
                    CoffeeState::CoffeeHeating => {
                        pid_1_data.enable = true;
                        pid_bg_data.enable = true;
                        pid_bg_data.target = 85.0;
                        led_state = LedState::SlowSine;

                        if state.heater_1_state == HeaterState::SteadyState
                            && state.heater_bg_state == HeaterState::SteadyState
                        {
                            state.coffee_state = CoffeeState::Ready;
                        }
                    }
                    CoffeeState::Ready => {
                        led_state = LedState::On;
                        if interface.lever_switch {
                            if let Ok(mut pid_data_temp) =
                                pid_1_data_container_main.lock(Duration::ms(5))
                            {
                                previous_kp = pid_data_temp.kp;
                                previous_ki = pid_data_temp.ki;
                                previous_kd = pid_data_temp.kd;
                                previous_target = pid_data_temp.target;
                                // increase p value for extraction!
                                pid_data_temp.kp *= 2.0;
                                pid_data_temp.target += 2.0;
                            }
                            state.coffee_state = CoffeeState::PreInfuse;
                            timer = 0;
                        }
                        if state.heater_1_state != HeaterState::SteadyState
                            || state.heater_bg_state != HeaterState::SteadyState
                        {
                            state.coffee_state = CoffeeState::CoffeeHeating;
                        }
                    }
                    CoffeeState::PreInfuse => {
                        state.pump_state = PumpState::On(
                            (max_duty as f32 * (pump.pre_infuse_power / 100.0)) as u16,
                        );

                        state.valve_1_state = ValveState::Closed;
                        state.valve_2_state = ValveState::Closed;

                        led_state = LedState::SlowBlink;
                        // timer of 2.5s
                        if timer >= 5 {
                            state.coffee_state = CoffeeState::Extracting;
                            timer = 0;
                        }
                    }
                    CoffeeState::Extracting => {
                        state.pump_state =
                            PumpState::On((max_duty as f32 * (pump.extract_power / 100.0)) as u16);

                        // TODO: we need to set duty cycle to a high value for heating during extraction!

                        state.valve_1_state = ValveState::Closed;
                        state.valve_2_state = ValveState::Closed;

                        led_state = LedState::FastBlink;

                        // timeout of 30s
                        if timer >= extraction_time || lever.is_high() {
                            if let Ok(mut pid_data_temp) =
                                pid_1_data_container_main.lock(Duration::ms(5))
                            {
                                pid_data_temp.kp = previous_kp;
                                pid_data_temp.ki = previous_ki;
                                pid_data_temp.kd = previous_kd;
                                pid_data_temp.target = previous_target;
                            }
                            state.coffee_state = CoffeeState::Ready;
                        }
                    }
                    CoffeeState::SteamHeating => {
                        state.pump_state = PumpState::Off;

                        state.valve_1_state = ValveState::Closed;
                        state.valve_2_state = ValveState::Closed;

                        led_state = LedState::SlowSine;
                        if state.heater_1_state == HeaterState::SteadyState
                            && state.heater_2_state == HeaterState::SteadyState
                        {
                            state.coffee_state = CoffeeState::Ready;
                        }
                    }
                    CoffeeState::Steaming => {
                        state.pump_state =
                            PumpState::On((max_duty as f32 * (pump.steam_power / 100.0)) as u16);

                        led_state = LedState::FastBlink;

                        state.valve_1_state = ValveState::Closed;
                        state.valve_2_state = ValveState::Open;

                        // TODO: set exit condition here
                        state.coffee_state = CoffeeState::Ready;
                    }
                }

                match state.pump_state {
                    PumpState::Off => match pump_override {
                        None => {
                            bldc_pwm.set_duty(0);
                            bldc_en.set_high();
                            bldc_pwm.disable();
                        }
                        Some(state) => {
                            if state {
                                bldc_pwm.set_duty(
                                    (max_duty as f32 * (pump.extract_power / 100.0)) as u16,
                                );
                                bldc_en.set_low();
                                bldc_pwm.enable();
                            } else {
                                bldc_pwm.set_duty(0);
                                bldc_en.set_high();
                                bldc_pwm.disable();
                            }
                        }
                    },
                    PumpState::On(pwr) => match pump_override {
                        None => {
                            bldc_pwm.set_duty(pwr);
                            bldc_en.set_low();
                            bldc_pwm.enable();
                        }
                        Some(state) => {
                            if state {
                                bldc_pwm.set_duty(
                                    (max_duty as f32 * (pump.extract_power / 100.0)) as u16,
                                );
                                bldc_en.set_low();
                                bldc_pwm.enable();
                            } else {
                                bldc_pwm.set_duty(0);
                                bldc_en.set_high();
                                bldc_pwm.disable();
                            }
                        }
                    },
                }

                match state.valve_1_state {
                    ValveState::Open => match valve_1_override {
                        None => {
                            valve1_pin.set_low();
                        }
                        Some(state) => {
                            if !state {
                                valve1_pin.set_high();
                            } else {
                                valve1_pin.set_low();
                            }
                        }
                    },
                    ValveState::Closed => match valve_1_override {
                        None => {
                            valve1_pin.set_high();
                        }
                        Some(state) => {
                            if state {
                                valve1_pin.set_low();
                            } else {
                                valve1_pin.set_high();
                            }
                        }
                    },
                }

                match state.valve_2_state {
                    ValveState::Open => match valve_2_override {
                        None => {
                            valve2_pin.set_low();
                        }
                        Some(state) => {
                            if !state {
                                valve2_pin.set_high();
                            } else {
                                valve2_pin.set_low();
                            }
                        }
                    },
                    ValveState::Closed => match valve_2_override {
                        None => {
                            valve2_pin.set_high();
                        }
                        Some(state) => {
                            if state {
                                valve2_pin.set_low();
                            } else {
                                valve2_pin.set_high();
                            }
                        }
                    },
                }

                // update timer
                timer += 1;
                if timer >= 10000 {
                    timer = 1;
                }

                // send states
                if let Ok(mut pid_data_temp) = pid_1_data_container_main.lock(Duration::ms(5)) {
                    pid_data_temp.enable = pid_1_data.enable;
                }
                if let Ok(mut pid_data_temp) = pid_2_data_container_main.lock(Duration::ms(5)) {
                    pid_data_temp.enable = pid_2_data.enable;
                }
                if let Ok(mut pid_data_temp) = pid_bg_data_container_main.lock(Duration::ms(5)) {
                    pid_data_temp.enable = pid_bg_data.enable;
                    pid_data_temp.target = pid_bg_data.target;
                }
                if let Ok(mut led_state_temp) = led_state_container_main.lock(Duration::ms(5)) {
                    *led_state_temp = led_state.clone();
                }
                if let Ok(mut state_temp) = state_container_main.lock(Duration::ms(5)) {
                    *state_temp = state.clone();
                }
                if let Ok(mut interface_temp) = interface_data_container_main.lock(Duration::ms(5))
                {
                    interface_temp.button = interface.button;
                    interface_temp.lever_switch = interface.lever_switch;
                }

                CurrentTask::delay(Duration::ms(main_task_period));
            }
        })
        .unwrap();

    Task::new()
        .name("LED TASK")
        .stack_size(256)
        .priority(TaskPriority(0))
        .start(move || {
            let mut led_state = LedState::Off;
            let max_duty = led_pwm.get_max_duty();
            let mut count = 0;
            loop {
                if check_shutdown() {
                    break;
                }

                if let Ok(led_state_temp) = led_state_container_led.lock(Duration::ms(5)) {
                    led_state = led_state_temp.clone();
                }
                match led_state {
                    LedState::Off => {
                        led_pwm.disable();
                        led_pwm.set_duty(0);
                    }
                    LedState::On => {
                        led_pwm.enable();
                        led_pwm.set_duty(max_duty);
                    }
                    LedState::SlowSine => {
                        led_pwm.enable();
                        let val = max_duty
                            - (max_duty as f32 * (count as f32 / 1024.0 * PI).sin()) as u16; // LED1
                        led_pwm.set_duty(val);
                    }
                    LedState::FastBlink => {
                        led_pwm.enable();
                        led_pwm.set_duty(0);
                        CurrentTask::delay(Duration::ms(250));
                        led_pwm.set_duty(max_duty);
                        CurrentTask::delay(Duration::ms(240));
                    }
                    LedState::SlowBlink => {
                        led_pwm.enable();
                        led_pwm.set_duty(0);
                        CurrentTask::delay(Duration::ms(500));
                        led_pwm.set_duty(max_duty);
                        CurrentTask::delay(Duration::ms(490));
                    }
                }
                count += 10;
                CurrentTask::delay(Duration::ms(10));
            }
        })
        .unwrap();

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
    //             if check_shutdown() {
    //                 break;
    //             }
    //
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
    //                         CurrentTask::delay(Duration::ms(note.1 * tempo));
    //                     } else if note.0 == ' ' {
    //                         // 2.2 if ' ' tone is found disable output for one beat
    //                         buzz_pwm.disable(Channel::C4);
    //                         CurrentTask::delay(Duration::ms(tempo));
    //                     }
    //                 }
    //                 // 4. Silence for half a beat between notes
    //                 // 4.1 Disable the PWM output (silence)
    //                 buzz_pwm.disable(Channel::C4);
    //                 // 4.2 Keep the output off for half a beat between notes
    //                 CurrentTask::delay(Duration::ms(tempo / 2));
    //
    //                 // 5. Go back to 1.
    //             }
    //             let timestamp = timer.now().ticks();
    //             // usb_println(arrform!(64,"t = {:?}", timestamp).as_str());
    //             CurrentTask::delay(Duration::ms(1000));
    //         }
    //     }).unwrap();

    FreeRtosUtils::start_scheduler();
}

#[panic_handler]
unsafe fn custom_panic_handler(_info: &PanicInfo) -> ! {
    // Safety: the GPIO peripheral is static, and we're not racing anyone by
    // definition since we're in the process of panicking to a halt.

    // here we stop all other threads
    cortex_m::interrupt::free(|_| unsafe { SHUTDOWN = true });
    CurrentTask::delay(Duration::ms(100));

    let dp = Peripherals::steal();

    // Turn off the Heater, it's connected to pin PC6.
    // initialize ports
    dp.GPIOC.odr.write(|w| w.odr6().clear_bit());
    dp.GPIOE.odr.write(|w| w.odr4().clear_bit());

    // sets it into debug-mode and sets a breakpoint
    asm::bkpt();

    loop {}
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

#[no_mangle]
#[allow(non_snake_case, unused_variables)]
fn vApplicationStackOverflowHook(pxTask: FreeRtosTaskHandle, pcTaskName: FreeRtosCharPtr) {
    asm::bkpt();
}
