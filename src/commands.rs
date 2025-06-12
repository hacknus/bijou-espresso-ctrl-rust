use alloc::sync::Arc;

use arrform::{arrform, ArrForm};
use freertos_rust::{Duration, Queue};

use crate::usb_println;
use crate::utils::{Interface, MeasuredData, PidData, PumpData, State};

// Commands for the config task
#[derive(Debug, Clone, Copy)]
pub enum ConfigCommand {
    LoadAll,
    SaveAll,
    LoadPid,
    SavePid,
    LoadPump,
    SavePump,
    LoadInterface,
    SaveInterface,
}

#[derive(Copy, Clone, Debug)]
pub enum PumpCommand {
    PumpOverride(Option<bool>),
    PumpPower(i16),
    PumpHeatUpPower(i16),
    PumpPreInfusePower(i16),
    PumpCoffeePower(i16),
    PumpSteamPower(i16),
}

#[derive(Copy, Clone, Debug)]
pub enum HeaterCommand {
    Temperature(f32),
    Heating(bool),
    WindowSize(f32),
    PidP(f32),
    PidI(f32),
    PidD(f32),
    PidMaxVal(f32),
    Boiler1(Option<u32>),
}

#[derive(Copy, Clone, Debug)]
pub enum ValveCommand {
    Valve1(Option<bool>),
    Valve2(Option<bool>),
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
    heater_1_command_queue: &Arc<Queue<HeaterCommand>>,
    heater_2_command_queue: &Arc<Queue<HeaterCommand>>,
    heater_bg_command_queue: &Arc<Queue<HeaterCommand>>,
    pump_command_queue: &Arc<Queue<PumpCommand>>,
    valve_command_queue: &Arc<Queue<ValveCommand>>,
    config_command_queue: &Arc<Queue<ConfigCommand>>,
    hk: &mut bool,
    hk_period: &mut f32,
) {
    if cmd.contains("[CMD]") {
        let response = if cmd.contains("[CMD] setCoffeeTemperature=") {
            match extract_value(cmd) {
                None => cmd_has_no_value(),
                Some(val) => {
                    heater_1_command_queue
                        .send(
                            HeaterCommand::Temperature(val),
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
                        .unwrap();
                    config_command_queue
                        .send(
                            ConfigCommand::SaveInterface,
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
                        .unwrap();
                    arrform!(64, "[ACK] cmd OK, set coffee temperature = {}", val)
                }
            }
        } else if cmd.contains("[CMD] setSteamTemperature=") {
            cmd_invalid()
        } else if cmd.contains("[CMD] triggerExtraction=") {
            match extract_value(cmd) {
                None => cmd_has_no_value(),
                Some(val) => {
                    // TODO: trigger pump
                    arrform!(
                        64,
                        "[ACK] cmd OK, triggering extraction for = {} seconds",
                        val
                    )
                }
            }
        } else if cmd.contains("[CMD] set1P=") {
            match extract_value(cmd) {
                None => cmd_has_no_value(),
                Some(val) => {
                    heater_1_command_queue
                        .send(HeaterCommand::PidP(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    config_command_queue
                        .send(ConfigCommand::SavePid, Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    arrform!(64, "[ACK] cmd OK, set PID1 p value = {}", val)
                }
            }
        } else if cmd.contains("[CMD] set1I=") {
            match extract_value(cmd) {
                None => cmd_has_no_value(),
                Some(val) => {
                    heater_1_command_queue
                        .send(HeaterCommand::PidI(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    config_command_queue
                        .send(ConfigCommand::SavePid, Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    arrform!(64, "[ACK] cmd OK, set PID1 i value = {}", val)
                }
            }
        } else if cmd.contains("[CMD] set1D=") {
            match extract_value(cmd) {
                None => cmd_has_no_value(),
                Some(val) => {
                    heater_1_command_queue
                        .send(HeaterCommand::PidD(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    config_command_queue
                        .send(ConfigCommand::SavePid, Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    arrform!(64, "[ACK] cmd OK, set PID1 d value = {}", val)
                }
            }
        } else if cmd.contains("[CMD] set2P=") {
            match extract_value(cmd) {
                None => cmd_has_no_value(),
                Some(val) => {
                    heater_2_command_queue
                        .send(HeaterCommand::PidP(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    config_command_queue
                        .send(ConfigCommand::SavePid, Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    arrform!(64, "[ACK] cmd OK, set PID2 p value = {}", val)
                }
            }
        } else if cmd.contains("[CMD] set2I=") {
            match extract_value(cmd) {
                None => cmd_has_no_value(),
                Some(val) => {
                    heater_2_command_queue
                        .send(HeaterCommand::PidI(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    config_command_queue
                        .send(ConfigCommand::SavePid, Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    arrform!(64, "[ACK] cmd OK, set PID2 i value = {}", val)
                }
            }
        } else if cmd.contains("[CMD] set2D=") {
            match extract_value(cmd) {
                None => cmd_has_no_value(),
                Some(val) => {
                    heater_2_command_queue
                        .send(HeaterCommand::PidD(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    config_command_queue
                        .send(ConfigCommand::SavePid, Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    arrform!(64, "[ACK] cmd OK, set PID2 d value = {}", val)
                }
            }
        } else if cmd.contains("[CMD] setBGP=") {
            match extract_value(cmd) {
                None => cmd_has_no_value(),
                Some(val) => {
                    heater_bg_command_queue
                        .send(HeaterCommand::PidP(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    config_command_queue
                        .send(ConfigCommand::SavePid, Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    arrform!(64, "[ACK] cmd OK, set PID BG p value = {}", val)
                }
            }
        } else if cmd.contains("[CMD] setBGI=") {
            match extract_value(cmd) {
                None => cmd_has_no_value(),
                Some(val) => {
                    heater_bg_command_queue
                        .send(HeaterCommand::PidI(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    config_command_queue
                        .send(ConfigCommand::SavePid, Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    arrform!(64, "[ACK] cmd OK, set PID BG i value = {}", val)
                }
            }
        } else if cmd.contains("[CMD] setBGD=") {
            match extract_value(cmd) {
                None => cmd_has_no_value(),
                Some(val) => {
                    heater_bg_command_queue
                        .send(HeaterCommand::PidD(val), Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    config_command_queue
                        .send(ConfigCommand::SavePid, Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    arrform!(64, "[ACK] cmd OK, set PID BG d value = {}", val)
                }
            }
        } else if cmd.contains("[CMD] setWindowSize=") {
            match extract_value(cmd) {
                None => cmd_has_no_value(),
                Some(val) => {
                    heater_1_command_queue
                        .send(
                            HeaterCommand::WindowSize(val),
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
                        .unwrap();
                    config_command_queue
                        .send(ConfigCommand::SavePid, Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    arrform!(64, "[ACK] cmd OK, set window size = {} ms", val)
                }
            }
        } else if cmd.contains("[CMD] setPIDMaxVal=") {
            match extract_value(cmd) {
                None => cmd_has_no_value(),
                Some(val) => {
                    heater_1_command_queue
                        .send(
                            HeaterCommand::PidMaxVal(val),
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
                        .unwrap();
                    config_command_queue
                        .send(ConfigCommand::SavePid, Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    arrform!(64, "[ACK] cmd OK, set PID max value = {}", val)
                }
            }
        } else if cmd.contains("[CMD] setPumpCoffeePower=") {
            match extract_value(cmd) {
                None => cmd_has_no_value(),
                Some(val) => {
                    pump_command_queue
                        .send(
                            PumpCommand::PumpCoffeePower(val as i16),
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
                        .unwrap();
                    config_command_queue
                        .send(ConfigCommand::SavePump, Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    arrform!(64, "[ACK] cmd OK, set pump coffee power = {}", val)
                }
            }
        } else if cmd.contains("[CMD] setPumpSteamPower=") {
            match extract_value(cmd) {
                None => cmd_has_no_value(),
                Some(val) => {
                    pump_command_queue
                        .send(
                            PumpCommand::PumpSteamPower(val as i16),
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
                        .unwrap();
                    config_command_queue
                        .send(ConfigCommand::SavePump, Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    arrform!(64, "[ACK] cmd OK, set pump steam power = {}", val)
                }
            }
        } else if cmd.contains("[CMD] setPumpPreInfusePower=") {
            match extract_value(cmd) {
                None => cmd_has_no_value(),
                Some(val) => {
                    pump_command_queue
                        .send(
                            PumpCommand::PumpPreInfusePower(val as i16),
                            Duration::ms(CMD_QUEUE_TIMEOUT),
                        )
                        .unwrap();
                    config_command_queue
                        .send(ConfigCommand::SavePump, Duration::ms(CMD_QUEUE_TIMEOUT))
                        .unwrap();
                    arrform!(64, "[ACK] cmd OK, set pump pre-infuse power = {}", val)
                }
            }
        } else if cmd.contains("[CMD] setPumpHeatUpPower=") {
            cmd_invalid()
        } else if cmd.contains("[CMD] OverrideValve1=Open") {
            valve_command_queue
                .send(
                    ValveCommand::Valve1(Some(true)),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] OverrideValve1=Close") {
            valve_command_queue
                .send(
                    ValveCommand::Valve1(Some(false)),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] ClearValve1Override") {
            valve_command_queue
                .send(ValveCommand::Valve1(None), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] OverrideValve2=Open") {
            valve_command_queue
                .send(
                    ValveCommand::Valve2(Some(true)),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] OverrideValve2=Close") {
            valve_command_queue
                .send(
                    ValveCommand::Valve2(Some(false)),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] ClearValve2Override") {
            valve_command_queue
                .send(ValveCommand::Valve2(None), Duration::ms(CMD_QUEUE_TIMEOUT))
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] startHeating1") {
            heater_1_command_queue
                .send(
                    HeaterCommand::Heating(true),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] stopHeating1") {
            heater_1_command_queue
                .send(
                    HeaterCommand::Heating(false),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] startHeating2") {
            heater_2_command_queue
                .send(
                    HeaterCommand::Heating(true),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] stopHeating2") {
            heater_2_command_queue
                .send(
                    HeaterCommand::Heating(false),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] startHeatingBG") {
            heater_bg_command_queue
                .send(
                    HeaterCommand::Heating(true),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] stopHeatingBG") {
            heater_bg_command_queue
                .send(
                    HeaterCommand::Heating(false),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] startHeatingAll") {
            heater_1_command_queue
                .send(
                    HeaterCommand::Heating(true),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            heater_2_command_queue
                .send(
                    HeaterCommand::Heating(true),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            heater_bg_command_queue
                .send(
                    HeaterCommand::Heating(true),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] stopHeatingAll") {
            heater_1_command_queue
                .send(
                    HeaterCommand::Heating(false),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            heater_2_command_queue
                .send(
                    HeaterCommand::Heating(false),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            heater_bg_command_queue
                .send(
                    HeaterCommand::Heating(false),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] startPump") {
            pump_command_queue
                .send(
                    PumpCommand::PumpOverride(Some(true)),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] stopPump") {
            pump_command_queue
                .send(
                    PumpCommand::PumpOverride(Some(false)),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] stopAll") {
            pump_command_queue
                .send(
                    PumpCommand::PumpOverride(Some(false)),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            heater_1_command_queue
                .send(
                    HeaterCommand::Heating(false),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            heater_2_command_queue
                .send(
                    HeaterCommand::Heating(false),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            heater_bg_command_queue
                .send(
                    HeaterCommand::Heating(false),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] clearOverridePump") {
            pump_command_queue
                .send(
                    PumpCommand::PumpOverride(None),
                    Duration::ms(CMD_QUEUE_TIMEOUT),
                )
                .unwrap();
            cmd_ok()
        } else if cmd.contains("[CMD] disableHK") {
            *hk = false;
            cmd_ok()
        } else if cmd.contains("[CMD] enableHK") {
            *hk = true;
            cmd_ok()
        } else if cmd.contains("[CMD] setHKRate=") {
            match extract_value(cmd) {
                Some(r) => {
                    if r <= 100.0 {
                        *hk_period = 1000.0 / r;
                        arrform!(64, "[ACK] cmd OK, set HK rate = {}", r)
                    } else {
                        arrform!(64, "[ACK] error, value = {} is too large (> 100)", r)
                    }
                }
                None => cmd_has_no_value(),
            }
        } else if cmd.contains("Ping") {
            arrform!(64, "Pong")
        } else {
            // invalid command
            cmd_invalid()
        };
        usb_println(response.as_str());
    }
}

pub fn cmd_invalid() -> ArrForm<64> {
    arrform!(64, "[ACK] ERR command invalid")
}

pub fn cmd_ok() -> ArrForm<64> {
    arrform!(64, "[ACK] OK command valid")
}

pub fn cmd_has_no_value() -> ArrForm<64> {
    arrform!(64, "[ACK] error = no value found")
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
        state.coffee_state,
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
    let hk = arrform!(128, "[P] {:?}", temperatures.p);
    usb_println(hk.as_str());
}

pub fn send_housekeeping_for_pid_tuning(
    state: &State,
    temperatures: &MeasuredData,
    _interface: &Interface,
    pid_1: &PidData,
    pid_bg: &PidData,
    _pump: &PumpData,
    _msg: &str,
) {
    let hk = arrform!(
        164,
        "{:?}, {:.2}, {:.2}, {:.2}, {:.4}, {:.4}, {:.4}, {:.2}, {:.2}, {:.2}, {:.2}, {:.4}, {:.4}, {:.4}, {:.2}, {:.2}",
        state.coffee_state,
        temperatures.p.unwrap_or(0.0),
        temperatures.t2.unwrap_or(0.0),
        pid_1.target,
        pid_1.p,
        pid_1.i,
        pid_1.d,
        pid_1.pid_val,
        pid_1.duty_cycle,
        temperatures.t3.unwrap_or(0.0),
        pid_bg.target,
        pid_bg.p,
        pid_bg.i,
        pid_bg.d,
        pid_bg.pid_val,
        pid_bg.duty_cycle,
    );
    usb_println(hk.as_str());
}
