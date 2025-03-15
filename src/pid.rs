pub const STEADY_STATE_BOUNDS: f32 = 1.0;

pub struct PID {
    pub enabled: bool,
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub val: f32,
    pub duty_cycle: f32,
    error: f32,
    prev_time: f32,
    pub target: f32,
    pub window_size: u32,
    pub max_val: f32,
}

#[allow(dead_code)]
impl PID {
    pub fn new() -> Self {
        PID {
            enabled: false,
            p: 0.0,
            i: 0.0,
            d: 0.0,
            kp: 1.0,
            ki: 0.0,
            kd: 2.0,
            val: 0.0,
            duty_cycle: 0.0,
            error: 0.0,
            prev_time: 0.0,
            target: 95.0,
            window_size: 500,
            max_val: 100.0,
        }
    }

    pub fn calculate(&mut self, temperature: f32, now: u32) -> f32 {
        let now = now as f32 / 1000.0;
        let current_error = self.target - temperature;

        self.p = self.kp * current_error;

        if self.enabled && self.duty_cycle < 1.0 && self.duty_cycle > 0.0 {
            self.i += self.ki * current_error / 1000.0; // scale factor of 1000 for convenience
        }
        self.d = self.kd * (current_error - self.error) / (now - self.prev_time);
        self.d = self.d.clamp(-0.1, 0.1);
        self.error = current_error;
        self.prev_time = now;
        self.val = self.p + self.i + self.d;
        self.val
    }

    pub fn get_heat_value(&mut self, temperature: f32, now: u32) -> f32 {
        let pid_val = self.calculate(temperature, now);
        if pid_val < 0.0 {
            self.duty_cycle = 0.0;
        } else {
            self.duty_cycle = pid_val;
        }
        // self.duty_cycle = self.target;
        self.duty_cycle
    }
}
