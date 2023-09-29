use alloc::sync::Arc;

use arrform::{arrform, ArrForm};
use freertos_rust::{Duration, Queue};

use crate::usb_println;
use crate::utils::{Interface, MeasuredData, PidData, State};

#[derive(Copy, Clone, Debug)]
pub enum Command {
    TriggerExtraction(f32),
    CoffeeTemperature(f32),
    SteamTemperature(f32),
    Pump(bool),
    PumpPower(i16),
    PumpHeatUpPower(i16),
    PumpPreInfusePower(i16),
    PumpCoffeePower(i16),
    PumpSteamPower(i16),
    Valve1(bool),
    Valve2(bool),
    Heating(bool),
    Boiler1(i16),
    Boiler2(i16),
    WindowSize(f32),
    PidP(f32),
    PidI(f32),
    PidD(f32),
    PidMaxVal(f32),
}

const CMD_QUEUE_TIMEOUT: u32 = 5;

fn extract_value(cmd: &str) -> Option<f32> {
    let mut start_index = 0;
    let mut end_index = 0;
    for (i, char) in cmd.char_indices() {
        if char == '=' {
            start_index = i + 1;
        } else if !"-0123456789.".contains(char) && start_index != 0 && start_index != i {
            end_index = i;
            break;
        }
    }
    if end_index == 0 {
        end_index = cmd.len();
    }
    let v = &cmd[start_index..end_index];
    v.parse::<f32>().ok()
}

pub fn extract_command(
    cmd: &str,
    command_queue: &Arc<Queue<Command>>,
    hk: &mut bool,
    hk_period: &mut f32,
) {
    if cmd.contains("[CMD]") {
        if cmd.contains("[CMD] setCoffeeTemperature=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(
                        arrform!(64, "[ACK] cmd OK, set coffee temperature = {}", val).as_str(),
                    );
                    command_queue
                        .send(
                            Command::CoffeeTemperature(val),
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] setSteamTemperature=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(
                        arrform!(64, "[ACK] cmd OK, set steam temperature = {}", val).as_str(),
                    );
                    command_queue
                        .send(
                            Command::SteamTemperature(val),
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] triggerExtraction=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(
                        arrform!(
                            64,
                            "[ACK] cmd OK, triggering extraction for = {} seconds",
                            val
                        )
                            .as_str(),
                    );
                    command_queue
                        .send(
                            Command::TriggerExtraction(val),
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] setP=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(arrform!(64, "[ACK] cmd OK, set p value = {}", val).as_str());
                    command_queue
                        .send(Command::PidP(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] setI=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(arrform!(64, "[ACK] cmd OK, set i value = {}", val).as_str());
                    command_queue
                        .send(Command::PidI(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] setD=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(arrform!(64, "[ACK] cmd OK, set d value = {}", val).as_str());
                    command_queue
                        .send(Command::PidD(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] setWindowSize=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(
                        arrform!(64, "[ACK] cmd OK, set window size = {} ms", val).as_str(),
                    );
                    command_queue
                        .send(Command::WindowSize(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] setPIDMaxVal=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(arrform!(64, "[ACK] cmd OK, set PID max value = {}", val).as_str());
                    command_queue
                        .send(Command::PidMaxVal(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] setPumpCoffeePower=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(
                        arrform!(64, "[ACK] cmd OK, set pump coffee power = {}", val).as_str(),
                    );
                    command_queue
                        .send(
                            Command::PumpCoffeePower(val as i16),
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] setPumpSteamPower=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(
                        arrform!(64, "[ACK] cmd OK, set pump steam power = {}", val).as_str(),
                    );
                    command_queue
                        .send(
                            Command::PumpSteamPower(val as i16),
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] setPumpPreInfusePower=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(
                        arrform!(64, "[ACK] cmd OK, set pump pre-infuse power = {}", val).as_str(),
                    );
                    command_queue
                        .send(
                            Command::PumpPreInfusePower(val as i16),
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] setPumpHeatUpPower=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(
                        arrform!(64, "[ACK] cmd OK, set pump heat up cycling power = {}", val)
                            .as_str(),
                    );
                    command_queue
                        .send(
                            Command::PumpHeatUpPower(val as i16),
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] openValve1") {
            command_queue
                .send(Command::Valve1(true), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] closeValve1") {
            command_queue
                .send(Command::Valve1(false), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] openValve2") {
            command_queue
                .send(Command::Valve2(true), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] closeValve2") {
            command_queue
                .send(Command::Valve2(false), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] startHeating") {
            command_queue
                .send(Command::Heating(true), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] stopHeating") {
            command_queue
                .send(Command::Heating(false), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] startPump") {
            command_queue
                .send(Command::Pump(true), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] stopPump") {
            command_queue
                .send(Command::Pump(false), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] stopAll") {
            command_queue
                .send(Command::Pump(false), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            command_queue
                .send(Command::Heating(false), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] disableHK") {
            *hk = false;
            cmd_ok();
        } else if cmd.contains("[CMD] enableHK") {
            *hk = true;
            cmd_ok();
        } else if cmd.contains("[CMD] setHKRate=") {
            match extract_value(cmd) {
                Some(r) => {
                    if r <= 100.0 {
                        *hk_period = 1000.0 / r;
                        usb_println(arrform!(64, "[ACK] cmd OK, set HK rate = {}", r).as_str());
                    } else {
                        usb_println(
                            arrform!(64, "[ACK] error, value = {} is too large (> 100)", r)
                                .as_str(),
                        );
                    }
                }
                None => {
                    cmd_has_no_value();
                }
            }
        } else {
            // invalid command
            cmd_failed();
        }
    }
}

pub fn cmd_failed() {
    usb_println("[ACK] ERR command invalid");
}

pub fn cmd_ok() {
    usb_println("[ACK] OK command valid");
}

pub fn cmd_has_no_value() {
    usb_println(arrform!(64, "[ACK] error = no value found").as_str());
}

pub fn send_housekeeping(
    state: &State,
    temperatures: &MeasuredData,
    interface: &Interface,
    output: &PidData,
    msg: &str,
) {
    let hk = arrform!(
        128,
        "[HK] {:?}, {}, {}, {}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {}, {}",
        state,
        interface.lever_switch,
        interface.steam_open,
        interface.water_low,
        temperatures.t1.unwrap_or(0.0),
        temperatures.t2.unwrap_or(0.0),
        temperatures.t3.unwrap_or(0.0),
        temperatures.t4.unwrap_or(0.0),
        temperatures.t5.unwrap_or(0.0),
        // TODO: pressure value!
        interface.coffee_temperature,
        output.p,
        output.i,
        output.d,
        output.pid_val,
        output.duty_cycle,
        msg
    );
    usb_println(hk.as_str());
}
