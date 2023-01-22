#[derive(Clone)]
pub struct MeasuredData {
    pub t1: Option<f32>,
    pub t2: Option<f32>,
    pub t3: Option<f32>,
    pub t4: Option<f32>,
    pub t5: Option<f32>,
    pub p: Option<f32>,
}

impl MeasuredData {
    pub fn new() -> Self {
        MeasuredData {
            t1: None,
            t2: None,
            t3: None,
            t4: None,
            t5: None,
            p: None,
        }
    }
}

#[derive(Clone)]
pub struct PumpData {
    pub heat_up_power: f32,
    pub pre_infuse_power: f32,
    pub steam_power: f32,
    pub extract_power: f32,
    pub extraction_timeout: f32,
    pub enable: bool,
}

impl PumpData {
    pub fn new() -> Self {
        PumpData {
            heat_up_power: 10.0,
            pre_infuse_power: 30.0,
            steam_power: 10.0,
            extract_power: 100.0,
            extraction_timeout: 20000.0,
            enable: false,
        }
    }
}

#[derive(Clone)]
pub struct PidData {
    pub p: Option<f32>,
    pub i: Option<f32>,
    pub d: Option<f32>,
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub enable: bool,
    pub pwm_val: Option<u32>,
}

impl PidData {
    pub fn new() -> Self {
        PidData {
            p: None,
            i: None,
            d: None,
            kp: 1.0,
            ki: 1.0,
            kd: 1.0,
            enable: false,
            pwm_val: None,
        }
    }
}

#[derive(Debug, Clone)]
pub enum State {
    Idle,
    CoffeeHeating,
    Ready,
    PreInfuse,
    Extracting,
    SteamHeating,
    Steaming,
}

#[derive(Debug, Clone)]
pub enum LedState {
    Off,
    On,
    SlowSine,
    SlowBlink,
    FastBlink,
}

#[derive(Clone)]
pub struct Interface {
    pub state: State,
    pub coffee_temperature: f32,
    pub steam_temperature: f32,
    pub lever_switch: bool,
    pub button: bool,
    pub steam_open: bool,
    pub water_low: bool,
    pub valve1: bool,
    pub valve2: bool,
}

impl Interface {
    pub fn new() -> Self {
        Interface {
            state: State::Idle,
            coffee_temperature: 92.0,
            steam_temperature: 120.0,
            lever_switch: false,
            button: false,
            steam_open: false,
            water_low: false,
            valve1: false,
            valve2: false,
        }
    }
}