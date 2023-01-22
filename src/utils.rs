#[derive(Clone)]
pub struct MeasuredData {
    pub t1 : Option<f32>,
    pub t2 : Option<f32>,
    pub t3 : Option<f32>,
    pub t4 : Option<f32>,
    pub t5 : Option<f32>,
    pub p : Option<f32>
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
pub struct PidData {
    pub p : Option<f32>,
    pub i : Option<f32>,
    pub d : Option<f32>,
    pub enable : bool,
    pub pwm_val : Option<u32>,
}

impl PidData {
    pub fn new() -> Self {
        PidData {
            p: None,
            i: None,
            d: None,
            enable: false,
            pwm_val: None,
        }
    }
}

#[derive(Debug, Clone)]
pub enum PumpState {
    Stop,
    Running,
}

#[derive(Debug, Clone)]
pub enum State {
    Idle,
    Heating,
    Ready,
    PreInfuse,
    Extracting
}

#[derive(Clone)]
pub struct Interface {
    pub pump_state : PumpState,
    pub state : State,
    pub lever_switch : bool,
    pub extraction_triggered : bool,
    pub steam_triggered : bool,
    pub steam_open : bool,
    pub water_low : bool,
}

impl Interface {
    pub fn new() -> Self {
        Interface {
            pump_state: PumpState::Stop,
            state: State::Idle,
            lever_switch: false,
            extraction_triggered: false,
            steam_triggered: false,
            steam_open: false,
            water_low: false,
        }
    }
}