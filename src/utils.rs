pub struct TemperatureData {
    pub t1 : Option<f32>,
    pub t2 : Option<f32>,
    pub t3 : Option<f32>,
    pub t4 : Option<f32>,
    pub t5 : Option<f32>,
}

impl TemperatureData {
    pub fn new() -> Self {
        TemperatureData{
            t1: None,
            t2: None,
            t3: None,
            t4: None,
            t5: None,
        }
    }
}

pub struct OutputData {
    pub p : Option<f32>,
    pub i : Option<f32>,
    pub d : Option<f32>,
    pub pwm_val : Option<u32>,
}

impl OutputData {
    pub fn new() -> Self {
        OutputData {
            p: None,
            i: None,
            d: None,
            pwm_val: None,
        }
    }
}

#[derive(Debug)]
pub enum State {
    Idle,
    Heating(f32),
    Ready,
    PreInfuse,
    Extracting(u32)
}

pub struct Interface {
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
            state: State::Idle,
            lever_switch: false,
            extraction_triggered: false,
            steam_triggered: false,
            steam_open: false,
            water_low: false,
        }
    }
}