use alloc::sync::Arc;

use arrform::{arrform, ArrForm};
use freertos_rust::{Duration, Mutex, Queue};

use crate::usb::usb_println;
use crate::utils::{Interface, MeasuredData, PidData, PumpData, State};

// ── Command enums sent to task queues ─────────────────────────────────────────

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
    ResetAll,
}

#[derive(Copy, Clone, Debug)]
pub enum PumpCommand {
    PumpOverride(Option<bool>),
    PumpHeatUpPower(i16),
    PumpPreInfusePower(i16),
    PumpCoffeePower(i16),
    PumpSteamPower(i16),
    PumpExtractionTimeout(f32),
    PumpPreInfuseTime(f32),
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
    Osr(u32),
    /// Manual duty-cycle override (0.0–1.0).  `None` clears the override and
    /// hands control back to the PID.
    OverrideDuty(Option<f32>),
}

#[derive(Copy, Clone, Debug)]
pub enum ValveCommand {
    Valve1(Option<bool>),
    Valve2(Option<bool>),
}

// ── Command context ───────────────────────────────────────────────────────────

pub struct CmdContext {
    pub heater_1_queue: Arc<Queue<HeaterCommand>>,
    pub heater_2_queue: Arc<Queue<HeaterCommand>>,
    pub heater_bg_queue: Arc<Queue<HeaterCommand>>,
    pub pump_queue: Arc<Queue<PumpCommand>>,
    pub valve_queue: Arc<Queue<ValveCommand>>,
    pub config_queue: Arc<Queue<ConfigCommand>>,
    /// Direct write-through for temperature setpoint updates.
    pub interface_container: Arc<Mutex<Interface>>,
    // Latest snapshots for getters (cloned at top of USB task loop).
    pub pid_1: PidData,
    pub pid_2: PidData,
    pub pid_bg: PidData,
    pub pump: PumpData,
    pub interface: Interface,
    pub state: State,
    pub temperatures: MeasuredData,
    // Housekeeping control – written back to the task loop after dispatch.
    pub hk: bool,
    pub hk_rate: f32,
}

// ── Table-driven dispatch ─────────────────────────────────────────────────────

const CMD_TIMEOUT_MS: u32 = 5;

struct CmdDef {
    prefix: &'static str,
    handler: fn(Option<f32>, &mut CmdContext),
}

// Queue send helpers
#[inline(always)]
fn q1(ctx: &mut CmdContext, c: HeaterCommand) {
    let _ = ctx.heater_1_queue.send(c, Duration::ms(CMD_TIMEOUT_MS));
}
#[inline(always)]
fn q2(ctx: &mut CmdContext, c: HeaterCommand) {
    let _ = ctx.heater_2_queue.send(c, Duration::ms(CMD_TIMEOUT_MS));
}
#[inline(always)]
fn qbg(ctx: &mut CmdContext, c: HeaterCommand) {
    let _ = ctx.heater_bg_queue.send(c, Duration::ms(CMD_TIMEOUT_MS));
}
#[inline(always)]
fn qpump(ctx: &mut CmdContext, c: PumpCommand) {
    let _ = ctx.pump_queue.send(c, Duration::ms(CMD_TIMEOUT_MS));
}
#[inline(always)]
fn qvalve(ctx: &mut CmdContext, c: ValveCommand) {
    let _ = ctx.valve_queue.send(c, Duration::ms(CMD_TIMEOUT_MS));
}
#[inline(always)]
fn qcfg(ctx: &mut CmdContext, c: ConfigCommand) {
    let _ = ctx.config_queue.send(c, Duration::ms(CMD_TIMEOUT_MS));
}

enum TempField {
    Coffee,
    BrewHead,
    Steam,
}

fn set_iface_temp(ctx: &mut CmdContext, field: TempField, val: f32) {
    if let Ok(mut iface) = ctx.interface_container.lock(Duration::ms(10)) {
        match field {
            TempField::Coffee => iface.coffee_temperature = val,
            TempField::BrewHead => iface.brew_head_temperature = val,
            TempField::Steam => iface.steam_temperature = val,
        }
    }
}

fn say(s: &str) {
    usb_println(s);
}
fn ok() {
    usb_println("[ACK] OK");
}
fn no_val() {
    usb_println("[ACK] ERR no value");
}
fn out_of_range(v: f32) {
    usb_println(arrform!(64, "[ACK] ERR value {} out of range", v).as_str());
}

// ── Range validators ──────────────────────────────────────────────────────────
fn valid_pid_gain(v: f32) -> bool {
    v.is_finite() && v >= 0.0 && v <= 10_000.0
}
fn valid_temp(v: f32) -> bool {
    v.is_finite() && v >= 0.0 && v <= 160.0
}
fn valid_power(v: f32) -> bool {
    v.is_finite() && v >= 0.0 && v <= 100.0
}
fn valid_timeout(v: f32) -> bool {
    v.is_finite() && v >= 100.0 && v <= 120_000.0
}
fn valid_pre_infuse(v: f32) -> bool {
    v.is_finite() && v >= 100.0 && v <= 30_000.0
}
fn valid_window(v: f32) -> bool {
    v.is_finite() && v >= 10.0 && v <= 10_000.0
}
fn valid_max_val(v: f32) -> bool {
    v.is_finite() && v >= 0.0 && v <= 1.5
}
fn valid_osr(v: f32) -> bool {
    v.is_finite() && v >= 1.0 && v <= 100.0
}

// ── Code-gen macros for the table ─────────────────────────────────────────────

/// Setter for a HeaterCommand f32 variant routed to queue `$q`.
macro_rules! set_pid {
    ($q:ident, $variant:ident, $valid:ident) => {
        |val, ctx| match val {
            Some(v) if $valid(v) => {
                $q(ctx, HeaterCommand::$variant(v));
                qcfg(ctx, ConfigCommand::SavePid);
                say(arrform!(64, "[ACK] = {:.6}", v).as_str());
            }
            Some(v) => out_of_range(v),
            None => no_val(),
        }
    };
}

/// Setter for the OSR (u32) heater parameter.
macro_rules! set_osr {
    ($q:ident) => {
        |val, ctx| match val {
            Some(v) if valid_osr(v) => {
                $q(ctx, HeaterCommand::Osr(v as u32));
                qcfg(ctx, ConfigCommand::SavePid);
                say(arrform!(64, "[ACK] = {}", v as u32).as_str());
            }
            Some(v) => out_of_range(v),
            None => no_val(),
        }
    };
}

/// Setter for a pump power value (i16 cast).
macro_rules! set_pump_pwr {
    ($variant:ident) => {
        |val, ctx| match val {
            Some(v) if valid_power(v) => {
                qpump(ctx, PumpCommand::$variant(v as i16));
                qcfg(ctx, ConfigCommand::SavePump);
                say(arrform!(64, "[ACK] = {}", v).as_str());
            }
            Some(v) => out_of_range(v),
            None => no_val(),
        }
    };
}

// ── Command table ─────────────────────────────────────────────────────────────

static CMD_TABLE: &[CmdDef] = &[
    // ── PID 1 – gains ────────────────────────────────────────────────────────
    CmdDef {
        prefix: "get1P",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] 1P = {:.6}", ctx.pid_1.kp).as_str());
        },
    },
    CmdDef {
        prefix: "set1P=",
        handler: set_pid!(q1, PidP, valid_pid_gain),
    },
    CmdDef {
        prefix: "get1I",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] 1I = {:.6}", ctx.pid_1.ki).as_str());
        },
    },
    CmdDef {
        prefix: "set1I=",
        handler: set_pid!(q1, PidI, valid_pid_gain),
    },
    CmdDef {
        prefix: "get1D",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] 1D = {:.6}", ctx.pid_1.kd).as_str());
        },
    },
    CmdDef {
        prefix: "set1D=",
        handler: set_pid!(q1, PidD, valid_pid_gain),
    },
    // ── PID 2 – gains ────────────────────────────────────────────────────────
    CmdDef {
        prefix: "get2P",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] 2P = {:.6}", ctx.pid_2.kp).as_str());
        },
    },
    CmdDef {
        prefix: "set2P=",
        handler: set_pid!(q2, PidP, valid_pid_gain),
    },
    CmdDef {
        prefix: "get2I",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] 2I = {:.6}", ctx.pid_2.ki).as_str());
        },
    },
    CmdDef {
        prefix: "set2I=",
        handler: set_pid!(q2, PidI, valid_pid_gain),
    },
    CmdDef {
        prefix: "get2D",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] 2D = {:.6}", ctx.pid_2.kd).as_str());
        },
    },
    CmdDef {
        prefix: "set2D=",
        handler: set_pid!(q2, PidD, valid_pid_gain),
    },
    // ── PID BG – gains ───────────────────────────────────────────────────────
    CmdDef {
        prefix: "getBGP",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] BGP = {:.6}", ctx.pid_bg.kp).as_str());
        },
    },
    CmdDef {
        prefix: "setBGP=",
        handler: set_pid!(qbg, PidP, valid_pid_gain),
    },
    CmdDef {
        prefix: "getBGI",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] BGI = {:.6}", ctx.pid_bg.ki).as_str());
        },
    },
    CmdDef {
        prefix: "setBGI=",
        handler: set_pid!(qbg, PidI, valid_pid_gain),
    },
    CmdDef {
        prefix: "getBGD",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] BGD = {:.6}", ctx.pid_bg.kd).as_str());
        },
    },
    CmdDef {
        prefix: "setBGD=",
        handler: set_pid!(qbg, PidD, valid_pid_gain),
    },
    // ── Per-heater: window size ───────────────────────────────────────────────
    CmdDef {
        prefix: "get1WindowSize",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] 1WindowSize = {}", ctx.pid_1.window_size).as_str());
        },
    },
    CmdDef {
        prefix: "set1WindowSize=",
        handler: set_pid!(q1, WindowSize, valid_window),
    },
    CmdDef {
        prefix: "get2WindowSize",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] 2WindowSize = {}", ctx.pid_2.window_size).as_str());
        },
    },
    CmdDef {
        prefix: "set2WindowSize=",
        handler: set_pid!(q2, WindowSize, valid_window),
    },
    CmdDef {
        prefix: "getBGWindowSize",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] BGWindowSize = {}", ctx.pid_bg.window_size).as_str());
        },
    },
    CmdDef {
        prefix: "setBGWindowSize=",
        handler: set_pid!(qbg, WindowSize, valid_window),
    },
    // ── Per-heater: max output ────────────────────────────────────────────────
    CmdDef {
        prefix: "get1PIDMaxVal",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] 1PIDMaxVal = {}", ctx.pid_1.max_val).as_str());
        },
    },
    CmdDef {
        prefix: "set1PIDMaxVal=",
        handler: set_pid!(q1, PidMaxVal, valid_max_val),
    },
    CmdDef {
        prefix: "get2PIDMaxVal",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] 2PIDMaxVal = {}", ctx.pid_2.max_val).as_str());
        },
    },
    CmdDef {
        prefix: "set2PIDMaxVal=",
        handler: set_pid!(q2, PidMaxVal, valid_max_val),
    },
    CmdDef {
        prefix: "getBGPIDMaxVal",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] BGPIDMaxVal = {}", ctx.pid_bg.max_val).as_str());
        },
    },
    CmdDef {
        prefix: "setBGPIDMaxVal=",
        handler: set_pid!(qbg, PidMaxVal, valid_max_val),
    },
    // ── Per-heater: oversampling ratio ────────────────────────────────────────
    CmdDef {
        prefix: "get1OSR",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] 1OSR = {}", ctx.pid_1.osr).as_str());
        },
    },
    CmdDef {
        prefix: "set1OSR=",
        handler: set_osr!(q1),
    },
    CmdDef {
        prefix: "get2OSR",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] 2OSR = {}", ctx.pid_2.osr).as_str());
        },
    },
    CmdDef {
        prefix: "set2OSR=",
        handler: set_osr!(q2),
    },
    CmdDef {
        prefix: "getBGOSR",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] BGOSR = {}", ctx.pid_bg.osr).as_str());
        },
    },
    CmdDef {
        prefix: "setBGOSR=",
        handler: set_osr!(qbg),
    },
    // ── Per-heater: live target ───────────────────────────────────────────────
    CmdDef {
        prefix: "get1Target",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] 1Target = {:.2}", ctx.pid_1.target).as_str());
        },
    },
    CmdDef {
        prefix: "get2Target",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] 2Target = {:.2}", ctx.pid_2.target).as_str());
        },
    },
    CmdDef {
        prefix: "getBGTarget",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] BGTarget = {:.2}", ctx.pid_bg.target).as_str());
        },
    },
    // ── Temperature setpoints ─────────────────────────────────────────────────
    CmdDef {
        prefix: "getCoffeeTemperature",
        handler: |_, ctx| {
            say(arrform!(
                64,
                "[ACK] coffeeT = {:.2}",
                ctx.interface.coffee_temperature
            )
            .as_str());
        },
    },
    CmdDef {
        prefix: "setCoffeeTemperature=",
        handler: |val, ctx| match val {
            Some(v) if valid_temp(v) => {
                q1(ctx, HeaterCommand::Temperature(v));
                set_iface_temp(ctx, TempField::Coffee, v);
                qcfg(ctx, ConfigCommand::SaveInterface);
                say(arrform!(64, "[ACK] coffeeT = {:.2}", v).as_str());
            }
            Some(v) => out_of_range(v),
            None => no_val(),
        },
    },
    CmdDef {
        prefix: "getBrewHeadTemperature",
        handler: |_, ctx| {
            say(arrform!(
                64,
                "[ACK] brewHeadT = {:.2}",
                ctx.interface.brew_head_temperature
            )
            .as_str());
        },
    },
    CmdDef {
        prefix: "setBrewHeadTemperature=",
        handler: |val, ctx| match val {
            Some(v) if valid_temp(v) => {
                qbg(ctx, HeaterCommand::Temperature(v));
                set_iface_temp(ctx, TempField::BrewHead, v);
                qcfg(ctx, ConfigCommand::SaveInterface);
                say(arrform!(64, "[ACK] brewHeadT = {:.2}", v).as_str());
            }
            Some(v) => out_of_range(v),
            None => no_val(),
        },
    },
    CmdDef {
        prefix: "getSteamTemperature",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] steamT = {:.2}", ctx.interface.steam_temperature).as_str());
        },
    },
    CmdDef {
        prefix: "setSteamTemperature=",
        handler: |val, ctx| match val {
            Some(v) if valid_temp(v) => {
                q2(ctx, HeaterCommand::Temperature(v));
                set_iface_temp(ctx, TempField::Steam, v);
                qcfg(ctx, ConfigCommand::SaveInterface);
                say(arrform!(64, "[ACK] steamT = {:.2}", v).as_str());
            }
            Some(v) => out_of_range(v),
            None => no_val(),
        },
    },
    // ── Pump powers ───────────────────────────────────────────────────────────
    CmdDef {
        prefix: "getPumpCoffeePower",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] coffeePwr = {}", ctx.pump.extract_power).as_str());
        },
    },
    CmdDef {
        prefix: "setPumpCoffeePower=",
        handler: set_pump_pwr!(PumpCoffeePower),
    },
    CmdDef {
        prefix: "getPumpSteamPower",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] steamPwr = {}", ctx.pump.steam_power).as_str());
        },
    },
    CmdDef {
        prefix: "setPumpSteamPower=",
        handler: set_pump_pwr!(PumpSteamPower),
    },
    CmdDef {
        prefix: "getPumpPreInfusePower",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] preInfusePwr = {}", ctx.pump.pre_infuse_power).as_str());
        },
    },
    CmdDef {
        prefix: "setPumpPreInfusePower=",
        handler: set_pump_pwr!(PumpPreInfusePower),
    },
    CmdDef {
        prefix: "getPumpHeatUpPower",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] heatUpPwr = {}", ctx.pump.heat_up_power).as_str());
        },
    },
    CmdDef {
        prefix: "setPumpHeatUpPower=",
        handler: set_pump_pwr!(PumpHeatUpPower),
    },
    // ── Timing parameters ─────────────────────────────────────────────────────
    CmdDef {
        prefix: "getExtractionTimeout",
        handler: |_, ctx| {
            say(arrform!(
                64,
                "[ACK] extractTimeout = {} ms",
                ctx.pump.extraction_timeout
            )
            .as_str());
        },
    },
    CmdDef {
        prefix: "setExtractionTimeout=",
        handler: |val, ctx| match val {
            Some(v) if valid_timeout(v) => {
                qpump(ctx, PumpCommand::PumpExtractionTimeout(v));
                qcfg(ctx, ConfigCommand::SavePump);
                say(arrform!(64, "[ACK] extractTimeout = {} ms", v).as_str());
            }
            Some(v) => out_of_range(v),
            None => no_val(),
        },
    },
    CmdDef {
        prefix: "getPreInfuseTime",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] preInfuseTime = {} ms", ctx.pump.pre_infuse_time).as_str());
        },
    },
    CmdDef {
        prefix: "setPreInfuseTime=",
        handler: |val, ctx| match val {
            Some(v) if valid_pre_infuse(v) => {
                qpump(ctx, PumpCommand::PumpPreInfuseTime(v));
                qcfg(ctx, ConfigCommand::SavePump);
                say(arrform!(64, "[ACK] preInfuseTime = {} ms", v).as_str());
            }
            Some(v) => out_of_range(v),
            None => no_val(),
        },
    },
    // ── Status / live readings ────────────────────────────────────────────────
    CmdDef {
        prefix: "getMachineState",
        handler: |_, ctx| {
            say(arrform!(64, "[ACK] state = {:?}", ctx.state.coffee_state).as_str());
        },
    },
    CmdDef {
        prefix: "getHeaterState",
        handler: |_, ctx| {
            say(arrform!(
                128,
                "[ACK] h1={:?} h2={:?} bg={:?}",
                ctx.state.heater_1_state,
                ctx.state.heater_2_state,
                ctx.state.heater_bg_state
            )
            .as_str());
        },
    },
    CmdDef {
        prefix: "getTemperatures",
        handler: |_, ctx| {
            say(arrform!(
                160,
                "[ACK] t1={:.2} t2={:.2} t3={:.2} t4={:.2} t5={:.2} p={:.2} i={:.2}",
                ctx.temperatures.t1.unwrap_or(0.0),
                ctx.temperatures.t2.unwrap_or(0.0),
                ctx.temperatures.t3.unwrap_or(0.0),
                ctx.temperatures.t4.unwrap_or(0.0),
                ctx.temperatures.t5.unwrap_or(0.0),
                ctx.temperatures.p.unwrap_or(0.0),
                ctx.temperatures.i.unwrap_or(0.0),
            )
            .as_str());
        },
    },
    // ── Dump all config ───────────────────────────────────────────────────────
    CmdDef {
        prefix: "dumpConfig",
        handler: |_, ctx| {
            say(arrform!(
                128,
                "[CFG] 1:  P={:.6} I={:.6} D={:.6} target={:.2} win={} maxVal={} osr={}",
                ctx.pid_1.kp,
                ctx.pid_1.ki,
                ctx.pid_1.kd,
                ctx.pid_1.target,
                ctx.pid_1.window_size,
                ctx.pid_1.max_val,
                ctx.pid_1.osr
            )
            .as_str());
            say(arrform!(
                128,
                "[CFG] 2:  P={:.6} I={:.6} D={:.6} target={:.2} win={} maxVal={} osr={}",
                ctx.pid_2.kp,
                ctx.pid_2.ki,
                ctx.pid_2.kd,
                ctx.pid_2.target,
                ctx.pid_2.window_size,
                ctx.pid_2.max_val,
                ctx.pid_2.osr
            )
            .as_str());
            say(arrform!(
                128,
                "[CFG] BG: P={:.6} I={:.6} D={:.6} target={:.2} win={} maxVal={} osr={}",
                ctx.pid_bg.kp,
                ctx.pid_bg.ki,
                ctx.pid_bg.kd,
                ctx.pid_bg.target,
                ctx.pid_bg.window_size,
                ctx.pid_bg.max_val,
                ctx.pid_bg.osr
            )
            .as_str());
            say(arrform!(
                128,
                "[CFG] temps: coffee={:.2} brewHead={:.2} steam={:.2}",
                ctx.interface.coffee_temperature,
                ctx.interface.brew_head_temperature,
                ctx.interface.steam_temperature
            )
            .as_str());
            say(arrform!(
                128,
                "[CFG] pump: coffee={} preInfuse={} steam={} heatUp={}",
                ctx.pump.extract_power,
                ctx.pump.pre_infuse_power,
                ctx.pump.steam_power,
                ctx.pump.heat_up_power
            )
            .as_str());
            say(arrform!(
                128,
                "[CFG] extractTimeout={} ms  preInfuseTime={} ms",
                ctx.pump.extraction_timeout,
                ctx.pump.pre_infuse_time
            )
            .as_str());
        },
    },
    // ── Heater on/off ─────────────────────────────────────────────────────────
    CmdDef {
        prefix: "startHeating1",
        handler: |_, ctx| {
            q1(ctx, HeaterCommand::Heating(true));
            ok();
        },
    },
    CmdDef {
        prefix: "stopHeating1",
        handler: |_, ctx| {
            q1(ctx, HeaterCommand::Heating(false));
            ok();
        },
    },
    CmdDef {
        prefix: "startHeating2",
        handler: |_, ctx| {
            q2(ctx, HeaterCommand::Heating(true));
            ok();
        },
    },
    CmdDef {
        prefix: "stopHeating2",
        handler: |_, ctx| {
            q2(ctx, HeaterCommand::Heating(false));
            ok();
        },
    },
    CmdDef {
        prefix: "startHeatingBG",
        handler: |_, ctx| {
            qbg(ctx, HeaterCommand::Heating(true));
            ok();
        },
    },
    CmdDef {
        prefix: "stopHeatingBG",
        handler: |_, ctx| {
            qbg(ctx, HeaterCommand::Heating(false));
            ok();
        },
    },
    CmdDef {
        prefix: "startHeatingAll",
        handler: |_, ctx| {
            q1(ctx, HeaterCommand::Heating(true));
            q2(ctx, HeaterCommand::Heating(true));
            qbg(ctx, HeaterCommand::Heating(true));
            ok();
        },
    },
    CmdDef {
        prefix: "stopHeatingAll",
        handler: |_, ctx| {
            q1(ctx, HeaterCommand::Heating(false));
            q2(ctx, HeaterCommand::Heating(false));
            qbg(ctx, HeaterCommand::Heating(false));
            ok();
        },
    },
    // ── Heater duty-cycle overrides ───────────────────────────────────────────
    CmdDef {
        prefix: "setH1Override=",
        handler: |v, ctx| match v {
            Some(d) => { q1(ctx, HeaterCommand::OverrideDuty(Some(d.clamp(0.0, 1.0)))); ok(); }
            None    => say("[ERR] setH1Override requires a value 0.0–1.0"),
        },
    },
    CmdDef {
        prefix: "clearH1Override",
        handler: |_, ctx| { q1(ctx, HeaterCommand::OverrideDuty(None)); ok(); },
    },
    CmdDef {
        prefix: "setH2Override=",
        handler: |v, ctx| match v {
            Some(d) => { q2(ctx, HeaterCommand::OverrideDuty(Some(d.clamp(0.0, 1.0)))); ok(); }
            None    => say("[ERR] setH2Override requires a value 0.0–1.0"),
        },
    },
    CmdDef {
        prefix: "clearH2Override",
        handler: |_, ctx| { q2(ctx, HeaterCommand::OverrideDuty(None)); ok(); },
    },
    CmdDef {
        prefix: "setHBGOverride=",
        handler: |v, ctx| match v {
            Some(d) => { qbg(ctx, HeaterCommand::OverrideDuty(Some(d.clamp(0.0, 1.0)))); ok(); }
            None    => say("[ERR] setHBGOverride requires a value 0.0–1.0"),
        },
    },
    CmdDef {
        prefix: "clearHBGOverride",
        handler: |_, ctx| { qbg(ctx, HeaterCommand::OverrideDuty(None)); ok(); },
    },
    // ── Pump control ──────────────────────────────────────────────────────────
    CmdDef {
        prefix: "startPump",
        handler: |_, ctx| {
            qpump(ctx, PumpCommand::PumpOverride(Some(true)));
            ok();
        },
    },
    CmdDef {
        prefix: "stopPump",
        handler: |_, ctx| {
            qpump(ctx, PumpCommand::PumpOverride(Some(false)));
            ok();
        },
    },
    CmdDef {
        prefix: "clearOverridePump",
        handler: |_, ctx| {
            qpump(ctx, PumpCommand::PumpOverride(None));
            ok();
        },
    },
    CmdDef {
        prefix: "stopAll",
        handler: |_, ctx| {
            qpump(ctx, PumpCommand::PumpOverride(Some(false)));
            q1(ctx, HeaterCommand::Heating(false));
            q2(ctx, HeaterCommand::Heating(false));
            qbg(ctx, HeaterCommand::Heating(false));
            ok();
        },
    },
    // ── Valve overrides ───────────────────────────────────────────────────────
    CmdDef {
        prefix: "OverrideValve1=Open",
        handler: |_, ctx| {
            qvalve(ctx, ValveCommand::Valve1(Some(true)));
            ok();
        },
    },
    CmdDef {
        prefix: "OverrideValve1=Close",
        handler: |_, ctx| {
            qvalve(ctx, ValveCommand::Valve1(Some(false)));
            ok();
        },
    },
    CmdDef {
        prefix: "ClearValve1Override",
        handler: |_, ctx| {
            qvalve(ctx, ValveCommand::Valve1(None));
            ok();
        },
    },
    CmdDef {
        prefix: "OverrideValve2=Open",
        handler: |_, ctx| {
            qvalve(ctx, ValveCommand::Valve2(Some(true)));
            ok();
        },
    },
    CmdDef {
        prefix: "OverrideValve2=Close",
        handler: |_, ctx| {
            qvalve(ctx, ValveCommand::Valve2(Some(false)));
            ok();
        },
    },
    CmdDef {
        prefix: "ClearValve2Override",
        handler: |_, ctx| {
            qvalve(ctx, ValveCommand::Valve2(None));
            ok();
        },
    },
    // ── Housekeeping ──────────────────────────────────────────────────────────
    CmdDef {
        prefix: "enableHK",
        handler: |_, ctx| {
            ctx.hk = true;
            ok();
        },
    },
    CmdDef {
        prefix: "disableHK",
        handler: |_, ctx| {
            ctx.hk = false;
            ok();
        },
    },
    CmdDef {
        prefix: "setHKRate=",
        handler: |val, ctx| match val {
            Some(r) if r > 0.0 && r <= 100.0 => {
                ctx.hk_rate = 1000.0 / r;
                say(arrform!(64, "[ACK] HKRate = {} Hz", r).as_str());
            }
            Some(r) => out_of_range(r),
            None => no_val(),
        },
    },
    // ── Persistent config ──────────────────────────────────────────────────────
    CmdDef {
        prefix: "saveConfig",
        handler: |_, ctx| {
            qcfg(ctx, ConfigCommand::SaveAll);
            say("[ACK] save queued");
        },
    },
    CmdDef {
        prefix: "loadConfig",
        handler: |_, ctx| {
            qcfg(ctx, ConfigCommand::LoadAll);
            say("[ACK] load queued");
        },
    },
    CmdDef {
        prefix: "loadPid",
        handler: |_, ctx| {
            qcfg(ctx, ConfigCommand::LoadPid);
            say("[ACK] load PID queued");
        },
    },
    CmdDef {
        prefix: "loadPump",
        handler: |_, ctx| {
            qcfg(ctx, ConfigCommand::LoadPump);
            say("[ACK] load pump queued");
        },
    },
    CmdDef {
        prefix: "loadInterface",
        handler: |_, ctx| {
            qcfg(ctx, ConfigCommand::LoadInterface);
            say("[ACK] load interface queued");
        },
    },
    CmdDef {
        prefix: "resetConfig",
        handler: |_, ctx| {
            qcfg(ctx, ConfigCommand::ResetAll);
            say("[ACK] reset queued");
        },
    },
    // ── Misc ──────────────────────────────────────────────────────────────────
    CmdDef {
        prefix: "Ping",
        handler: |_, _| {
            say("Pong");
        },
    },
];

// ── Value extraction ──────────────────────────────────────────────────────────

fn extract_value(cmd: &str) -> Option<f32> {
    let eq = cmd.find('=')?;
    let s = &cmd[eq + 1..];
    let end = s
        .find(|c: char| !c.is_ascii_digit() && c != '.' && c != '-')
        .unwrap_or(s.len());
    s[..end].parse::<f32>().ok()
}

// ── Public entry point ────────────────────────────────────────────────────────

pub fn extract_command(cmd: &str, ctx: &mut CmdContext) {
    if !cmd.contains("[CMD]") {
        return;
    }
    let body = match cmd.find("[CMD]") {
        Some(pos) => cmd[pos + 5..].trim_start(),
        None => return,
    };
    let val = extract_value(body);
    for def in CMD_TABLE {
        if body.starts_with(def.prefix) {
            (def.handler)(val, ctx);
            return;
        }
    }
    usb_println("[ACK] ERR command invalid");
}

// ── Housekeeping telemetry ─────────────────────────────────────────────────────

pub fn send_housekeeping(
    state: &State,
    temperatures: &MeasuredData,
    pid_1: &PidData,
    pid_bg: &PidData,
) {
    usb_println(
        arrform!(
            192,
            "{:?}, {:.2}, {:.2}, {:.2}, {:.4}, {:.4}, {:.4}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}",
            state.coffee_state,
            temperatures.p.unwrap_or(0.0),
            temperatures.i.unwrap_or(0.0),
            temperatures.t2.unwrap_or(0.0),
            pid_1.kp,
            pid_1.kd,
            pid_1.ki,
            pid_1.target,
            pid_1.pid_val,
            pid_1.duty_cycle * 100.0,
            temperatures.t3.unwrap_or(0.0),
            pid_bg.duty_cycle * 100.0,
        )
        .as_str(),
    );
    usb_println(
        arrform!(
            128,
            "[T] {:.2}, {:.2}, {:.2}, {:.2}, {:.2}",
            temperatures.t1.unwrap_or(0.0),
            temperatures.t2.unwrap_or(0.0),
            temperatures.t3.unwrap_or(0.0),
            temperatures.t4.unwrap_or(0.0),
            temperatures.t5.unwrap_or(0.0),
        )
        .as_str(),
    );
}
