use alloc::sync::Arc;

use arrform::{arrform, ArrForm};
use freertos_rust::{Duration, Queue};

use crate::usb_println;
use crate::utils::{Interface, MeasuredData, PidData, PumpData, State};

#[derive(Copy, Clone, Debug)]
pub enum PumpCommand {
    Pump(bool),
    PumpPower(i16),
    PumpHeatUpPower(i16),
    PumpPreInfusePower(i16),
    PumpCoffeePower(i16),
    PumpSteamPower(i16),
}

#[derive(Copy, Clone, Debug)]
pub enum HeaterCommand {
    CoffeeTemperature(f32),
    SteamTemperature(f32),
    Heating(bool),
    WindowSize(f32),
    PidP(f32),
    PidI(f32),
    PidD(f32),
    PidMaxVal(f32),
    Boiler1(Option<u32>),
    Boiler2(Option<u32>),
}

#[derive(Copy, Clone, Debug)]
pub enum ValveCommand {
    Valve1(bool),
    Valve2(bool),
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
    heater_command_queue: &Arc<Queue<HeaterCommand>>,
    pump_command_queue: &Arc<Queue<PumpCommand>>,
    valve_command_queue: &Arc<Queue<ValveCommand>>,
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
                    heater_command_queue
                        .send(
                            HeaterCommand::CoffeeTemperature(val),
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
                    heater_command_queue
                        .send(
                            HeaterCommand::SteamTemperature(val),
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
                    // TODO: trigger pump
                }
            }
        } else if cmd.contains("[CMD] setP=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(arrform!(64, "[ACK] cmd OK, set p value = {}", val).as_str());
                    heater_command_queue
                        .send(HeaterCommand::PidP(val), Duration::ms(CMD_QUEUE_TIMEOUT))
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
                    heater_command_queue
                        .send(HeaterCommand::PidI(val), Duration::ms(CMD_QUEUE_TIMEOUT))
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
                    heater_command_queue
                        .send(HeaterCommand::PidD(val), Duration::ms(CMD_QUEUE_TIMEOUT))
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
                    heater_command_queue
                        .send(
                            HeaterCommand::WindowSize(val),
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
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
                    heater_command_queue
                        .send(
                            HeaterCommand::PidMaxVal(val),
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
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
                    pump_command_queue
                        .send(
                            PumpCommand::PumpCoffeePower(val as i16),
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
                    pump_command_queue
                        .send(
                            PumpCommand::PumpSteamPower(val as i16),
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
                    pump_command_queue
                        .send(
                            PumpCommand::PumpPreInfusePower(val as i16),
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
                    pump_command_queue
                        .send(
                            PumpCommand::PumpHeatUpPower(val as i16),
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
                        .unwrap();
                }
            }
        } else if cmd.contains("[CMD] openValve1") {
            valve_command_queue
                .send(ValveCommand::Valve1(true), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] closeValve1") {
            valve_command_queue
                .send(ValveCommand::Valve1(false), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] openValve2") {
            valve_command_queue
                .send(ValveCommand::Valve2(true), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] closeValve2") {
            valve_command_queue
                .send(ValveCommand::Valve2(false), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] startHeating") {
            heater_command_queue
                .send(
                    HeaterCommand::Heating(true),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] stopHeating") {
            heater_command_queue
                .send(
                    HeaterCommand::Heating(false),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] startPump") {
            pump_command_queue
                .send(PumpCommand::Pump(true), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] stopPump") {
            pump_command_queue
                .send(PumpCommand::Pump(false), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok();
        } else if cmd.contains("[CMD] stopAll") {
            pump_command_queue
                .send(PumpCommand::Pump(false), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            heater_command_queue
                .send(
                    HeaterCommand::Heating(false),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
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
    pump: &PumpData,
    msg: &str,
) {
    let hk = arrform!(
        128,
        "[SYS] {:?}, {}, {}, {}, {}, {}, {}, {}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {}, {}",
        state,
        interface.lever_switch,
        interface.steam_open,
        interface.water_low,
        pump.heat_up_power,
        pump.pre_infuse_power,
        pump.steam_power,
        pump.extract_power,
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
    let hk = arrform!(
        128,
        "[T] {:.2}, {:.2}, {:.2}, {:.2}, {:.2}",
        temperatures.t1.unwrap_or(0.0),
        temperatures.t2.unwrap_or(0.0),
        temperatures.t3.unwrap_or(0.0),
        temperatures.t4.unwrap_or(0.0),
        temperatures.t5.unwrap_or(0.0),
    );
    usb_println(hk.as_str());
}
