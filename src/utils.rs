#[derive(Clone, Default)]
pub struct MeasuredData {
    pub t1: Option<f32>,
    pub t2: Option<f32>,
    pub t3: Option<f32>,
    pub t4: Option<f32>,
    pub t5: Option<f32>,
    pub p: Option<f32>,
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

impl Default for PumpData {
    fn default() -> Self {
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
    pub target: f32,
    pub current_temperature: Option<f32>,
    pub sensor: usize,
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub window_size: u32,
    pub max_val: f32,
    pub osr: u32,
    pub enable: bool,
    pub reset_i: bool,
    pub pid_val: f32,
    pub duty_cycle: u32,
}

impl Default for PidData {
    fn default() -> Self {
        PidData {
            target: 100.0,
            current_temperature: None,
            sensor: 0,
            p: 0.0,
            i: 0.0,
            d: 0.0,
            kp: 1.0,
            ki: 1.0,
            kd: 1.0,
            window_size: 500,
            max_val: 0.0,
            osr: 1,
            enable: false,
            reset_i: false,
            pid_val: 0.0,
            duty_cycle: 0,
        }
    }
}
#[allow(dead_code)]
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
    pub coffee_temperature: f32,
    pub steam_temperature: f32,
    pub trigger_extraction: bool,
    pub lever_switch: bool,
    pub button: bool,
    pub steam_open: bool,
    pub water_low: bool,
    pub valve1: bool,
    pub valve2: bool,
}

impl Default for Interface {
    fn default() -> Self {
        Interface {
            coffee_temperature: 92.0,
            steam_temperature: 120.0,
            trigger_extraction: false,
            lever_switch: false,
            button: false,
            steam_open: false,
            water_low: false,
            valve1: false,
            valve2: false,
        }
    }
}
