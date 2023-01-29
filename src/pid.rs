
pub struct PID {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub val: f32,
    pub duty_cycle: u32,
    error: f32,
    prev_time: u32,
    pub target: f32,
    pub window_size: u32,
    pub max_val: f32
}


impl PID {
    pub fn new() -> Self {
        PID {
            p: 0.0,
            i: 0.0,
            d: 0.0,
            kp: 1.0,
            ki: 1.0,
            kd: 1.0,
            val: 0.0,
            duty_cycle: 0,
            error: 0.0,
            prev_time: 0,
            target: 90.0,
            window_size: 500,
            max_val: 100.0,
        }
    }

    pub fn calculate(&mut self, temperature: f32, now: u32) -> f32 {
        let current_error = self.target - temperature;
        self.p = self.kp * current_error;
        self.i += self.ki * current_error;
        self.d = self.kd * (current_error - self.error) / (now - self.prev_time) as f32;
        self.error = current_error;
        self.prev_time = now;
        self.val = self.p + self.i + self.d;
        self.val
    }

    pub fn get_heat_value(&mut self, temperature: f32, now: u32) -> u32 {
        let mut pid_val = self.calculate(temperature, now);
        if pid_val < 0.0 {
            pid_val = 0.0;
        } else if pid_val > self.max_val {
            pid_val = self.max_val;
        }
        self.duty_cycle = (pid_val / self.max_val * self.window_size as f32) as u32;
        self.duty_cycle
    }
}