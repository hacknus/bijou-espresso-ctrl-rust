use arrform::{arrform, ArrForm};
use cortex_m::asm::delay;
use crate::{usb_println};
use crate::utils::{Interface, OutputData, State, TemperatureData};

fn get_last_index(cmd: &str) -> usize {
    let mut index: usize = 0;
    for i in 0..cmd.len() {
        if &cmd[i..i + 2] == "  " {
            index = i;
            break;
        }
        index = i;
    }
    index
}

pub fn extract_command(
    state: &mut State,
    cmd: &str,
    hk: &mut bool,
    hk_rate: &mut f32,
) {
    if cmd.contains("[CMD]") {
        if cmd.contains("[CMD] setCoffeeTemperature=") {
            let val: f32;
            let index = get_last_index(cmd);
            usb_println(arrform!(64,"index = {}", index).as_str());
            match cmd[27..index].parse::<f32>() {
                Ok(s) => {
                    val = s;
                    *state = State::Heating(val);
                    usb_println(arrform!(64,"[ACK] cmd OK, set coffee temperature = {}", s).as_str());
                }
                Err(err) => {
                    usb_println(arrform!(64,"[ACK] error = {:?}", err).as_str());
                }
            }
        } else if cmd.contains("[CMD] setSteamTemperature=") {
            let val: f32;
            let index = get_last_index(cmd);
            match cmd[26..index].parse::<f32>() {
                Ok(s) => {
                    val = s;
                    *state = State::Heating(val);
                    usb_println(arrform!(64,"[ACK] cmd OK, set steam temperature = {}", s).as_str());
                }
                Err(err) => {
                    usb_println(arrform!(64,"[ACK] error = {:?}", err).as_str());
                }
            }
        } else if cmd.contains("[CMD] triggerExtraction=") {
            let val: f32;
            let index = get_last_index(cmd);
            match cmd[24..index].parse::<f32>() {
                Ok(s) => {
                    val = s;
                    // 400 Hz corresponds to 1 mm/s movement
                    *state = State::Extracting(val as u32);
                    usb_println(arrform!(64,"[ACK] cmd OK, start extracting for {} seconds", s).as_str());
                }
                Err(err) => {
                    usb_println(arrform!(64,"[ACK] error = {:?}", err).as_str());
                }
            }
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
            let index = get_last_index(cmd);
            match cmd[18..index].parse::<f32>() {
                Ok(r) => {
                    if r <= 10.0 {
                        *hk_rate = 1000.0 / r;
                        usb_println(arrform!(64,"[ACK] cmd OK, val = {}", r).as_str());

                    } else {
                        usb_println(arrform!(64,"[ACK] error, value = {} is too large (> 10)", r).as_str());
                    }
                }
                Err(err) => {
                    usb_println(arrform!(64,"[ACK] error = {:?}", err).as_str());
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
    temperatures: &TemperatureData,
    interface: &Interface,
    output: &OutputData,
    msg: &str,
) {
    let hk = arrform!(
        128,
        "[HK] {:?}, {}, {}, {}, {}, {}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {}, {}",
        interface.state,
        interface.extraction_triggered,
        interface.lever_switch,
        interface.steam_open,
        interface.steam_triggered,
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
