use freertos_rust::freertos_rs_xTaskGetTickCount;
use stm32f4xx_hal::prelude::_fugit_DurationExtU32;

pub struct PID {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    error: f32,
    prev_time: u32,
    pub target: f32,
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
            error: 0.0,
            prev_time: 0,
            target: 90.0,
        }
    }

    pub fn calculate(&mut self, temperature: f32, now: u32) -> f32 {
        let current_error = self.target - temperature;
        self.p = self.kp * current_error;
        self.i += self.ki * current_error;
        self.d = self.kd * (current_error - self.error) / (now - self.prev_time) as f32;
        self.error = current_error;
        self.prev_time = now;
        self.p + self.i + self.d
    }
}