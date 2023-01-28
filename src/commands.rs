use arrform::{arrform, ArrForm};
use cortex_m::asm::delay;
use crate::{usb_println};
use crate::utils::{Interface, PidData, MeasuredData, State};

fn extract_value(cmd: &str) -> Option<f32> {
    let mut start_index = 0;
    let mut end_index = 0;
    for (i, char) in cmd.bytes().enumerate() {
        if char == b'=' {
            start_index = i + 1;
        } else if !b"-0123456789.".contains(&char) && start_index != 0 && start_index != i {
            end_index = i;
            break;
        }
    }
    if end_index == 0 {
        end_index = cmd.len();
    }
    let v = &cmd[start_index..end_index];
    match v.parse::<f32>() {
        Ok(val) => { Some(val) }
        Err(_) => { None }
    }
}

pub fn extract_command(
    state: &mut State,
    cmd: &str,
    hk: &mut bool,
    hk_period: &mut f32,
) {
    if cmd.contains("[CMD]") {
        if cmd.contains("[CMD] setCoffeeTemperature=") {
            match extract_value(cmd) {
                None => {
                    usb_println(arrform!(64,"[ACK] error = no value found").as_str());
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set coffee temperature = {}", val).as_str());
                }
            }
        } else if cmd.contains("[CMD] setSteamTemperature=") {
            match extract_value(cmd) {
                None => {
                    usb_println(arrform!(64,"[ACK] error = no value found").as_str());
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set steam temperature = {}", val).as_str());
                }
            }
        } else if cmd.contains("[CMD] triggerExtraction=") {
            match extract_value(cmd) {
                None => {
                    usb_println(arrform!(64,"[ACK] error = no value found").as_str());
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, triggering extraction for = {} seconds", val).as_str());
                }
            }
        } else if cmd.contains("[CMD] setP=") {
            match extract_value(cmd) {
                None => {
                    usb_println(arrform!(64,"[ACK] error = no value found").as_str());
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set p value = {}", val).as_str());
                }
            }
        } else if cmd.contains("[CMD] setI=") {
            match extract_value(cmd) {
                None => {
                    usb_println(arrform!(64,"[ACK] error = no value found").as_str());
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set i value = {}", val).as_str());
                }
            }
        } else if cmd.contains("[CMD] setD=") {
            match extract_value(cmd) {
                None => {
                    usb_println(arrform!(64,"[ACK] error = no value found").as_str());
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set d value = {}", val).as_str());
                }
            }
        } else if cmd.contains("[CMD] setWindowSize=") {
            match extract_value(cmd) {
                None => {
                    usb_println(arrform!(64,"[ACK] error = no value found").as_str());
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set window size = {}", val).as_str());
                }
            }
        } else if cmd.contains("[CMD] setPIDMaxVal=") {
            match extract_value(cmd) {
                None => {
                    usb_println(arrform!(64,"[ACK] error = no value found").as_str());
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set PID max value = {}", val).as_str());
                }
            }
        } else if cmd.contains("[CMD] setPumpSpeed=") {
            match extract_value(cmd) {
                None => {
                    usb_println(arrform!(64,"[ACK] error = no value found").as_str());
                }
                Some(val) => {
                    usb_println(arrform!(64,"[ACK] cmd OK, set pump speed = {}", val).as_str());
                }
            }
        } else if cmd.contains("[CMD] openValve1") {
            cmd_ok();
        } else if cmd.contains("[CMD] closeValve1") {
            cmd_ok();
        } else if cmd.contains("[CMD] openValve2") {
            cmd_ok();
        } else if cmd.contains("[CMD] closeValve2") {
            cmd_ok();
        } else if cmd.contains("[CMD] stopHeating") {
            cmd_ok();
        } else if cmd.contains("[CMD] stopPump") {
            cmd_ok();
        } else if cmd.contains("[CMD] stopAll") {
            cmd_ok();
        } else if cmd.contains("[CMD] disable hk") {
            *hk = false;
            cmd_ok();
        } else if cmd.contains("[CMD] enable hk") {
            *hk = true;
            cmd_ok();
        } else if cmd.contains("[CMD] set hk rate=") {
            match extract_value(cmd) {
                Some(r) => {
                    if r <= 100.0 {
                        *hk_period = 1000.0 / r;
                        usb_println(arrform!(64,"[ACK] cmd OK, val = {}", r).as_str());
                    } else {
                        usb_println(arrform!(64,"[ACK] error, value = {} is too large (> 100)", r).as_str());
                    }
                }
                None => {
                    usb_println(arrform!(64,"[ACK] error = no value found").as_str());
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

pub fn send_housekeeping(
    temperatures: &MeasuredData,
    interface: &Interface,
    output: &PidData,
    msg: &str,
) {
    let hk = arrform!(
        128,
        "[HK] {:?}, {}, {}, {}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {}, {}",
        interface.state,
        interface.lever_switch,
        interface.steam_open,
        interface.water_low,
        temperatures.t1.unwrap_or(0.0),
        temperatures.t2.unwrap_or(0.0),
        temperatures.t3.unwrap_or(0.0),
        temperatures.t4.unwrap_or(0.0),
        temperatures.t5.unwrap_or(0.0),
        // TODO: pressure value!
        output.p.unwrap_or(0.0),
        output.i.unwrap_or(0.0),
        output.d.unwrap_or(0.0),
        output.pwm_val.unwrap_or(0),
        msg
    );
    usb_println(hk.as_str());
}
