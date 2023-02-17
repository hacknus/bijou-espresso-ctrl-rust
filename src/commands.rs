use arrform::{arrform, ArrForm};
use crate::{usb_println};
use crate::utils::{Interface, PidData, MeasuredData, State, PumpData};

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
    interface: &mut Interface,
    output: &mut PidData,
    pump: &mut PumpData,
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
                    usb_println(arrform!(64,"[ACK] cmd OK, set coffee temperature = {}", val).as_str());
                    interface.coffee_temperature = val;
                }
            }
        } else if cmd.contains("[CMD] setSteamTemperature=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set steam temperature = {}", val).as_str());
                    interface.steam_temperature = val;
                }
            }
        } else if cmd.contains("[CMD] triggerExtraction=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, triggering extraction for = {} seconds", val).as_str());
                    interface.trigger_extraction = true;
                    pump.extraction_timeout = val;
                }
            }
        } else if cmd.contains("[CMD] setP=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set p value = {}", val).as_str());
                    output.kp = val;
                }
            }
        } else if cmd.contains("[CMD] setI=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set i value = {}", val).as_str());
                    output.ki = val;
                }
            }
        } else if cmd.contains("[CMD] setD=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set d value = {}", val).as_str());
                    output.kd = val;
                }
            }
        } else if cmd.contains("[CMD] setWindowSize=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set window size = {} ms", val).as_str());
                    output.window_size = val as u32;
                }
            }
        } else if cmd.contains("[CMD] setPIDMaxVal=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set PID max value = {}", val).as_str());
                    output.max_val = val;
                }
            }
        } else if cmd.contains("[CMD] setPumpCoffeePower=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set pump coffee power = {}", val).as_str());
                    pump.extract_power = val;
                }
            }
        } else if cmd.contains("[CMD] setPumpSteamPower=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set pump steam power = {}", val).as_str());
                    pump.steam_power = val;
                }
            }
        } else if cmd.contains("[CMD] setPumpPreInfusePower=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set pump pre-infuse power = {}", val).as_str());
                    pump.pre_infuse_power = val;
                }
            }
        } else if cmd.contains("[CMD] setPumpHeatUpPower=") {
            match extract_value(cmd) {
                None => {
                    cmd_has_no_value();
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set pump heat up cycling power = {}", val).as_str());
                    pump.heat_up_power = val;
                }
            }
        } else if cmd.contains("[CMD] openValve1") {
            interface.valve1 = true;
            cmd_ok();
        } else if cmd.contains("[CMD] closeValve1") {
            interface.valve1 = false;
            cmd_ok();
        } else if cmd.contains("[CMD] openValve2") {
            interface.valve2 = true;
            cmd_ok();
        } else if cmd.contains("[CMD] closeValve2") {
            interface.valve2 = false;
            cmd_ok();
        } else if cmd.contains("[CMD] startHeating") {
            output.enable = true;
            cmd_ok();
        } else if cmd.contains("[CMD] stopHeating") {
            output.enable = false;
            cmd_ok();
        } else if cmd.contains("[CMD] startPump") {
            pump.enable = true;
            cmd_ok();
        } else if cmd.contains("[CMD] stopPump") {
            pump.enable = false;
            cmd_ok();
        } else if cmd.contains("[CMD] stopAll") {
            pump.enable = false;
            output.enable = false;
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
                        usb_println(arrform!(64,"[ACK] cmd OK, set HK rate = {}", r).as_str());
                    } else {
                        usb_println(arrform!(64,"[ACK] error, value = {} is too large (> 100)", r).as_str());
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
    usb_println(arrform!(64,"[ACK] error = no value found").as_str());
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
        "[HK] {:?}, {}, {}, {}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {}, {}, {}",
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
        output.p,
        output.i,
        output.d,
        output.pid_val,
        output.duty_cycle,
        msg
    );
    usb_println(hk.as_str());
}
