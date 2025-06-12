pub const MAV_LENGTH: usize = 12;

pub struct Mav {
    pub history: [f32; MAV_LENGTH],
    write_idx: usize,
}

impl Mav {
    pub fn new() -> Mav {
        Mav {
            history: [0.0; MAV_LENGTH],
            write_idx: 0,
        }
    }

    pub fn oldest(&mut self) -> f32 {
        self.history[self.write_idx % MAV_LENGTH]
    }

    pub fn newest(&mut self) -> f32 {
        let index = if self.write_idx == 0 {
            MAV_LENGTH - 1
        } else {
            self.write_idx - 1
        };
        self.history[index]
    }

    pub fn spread(&mut self) -> f32 {
        let spread = self.oldest() - self.newest();
        if spread < 0.0 {
            -spread
        } else {
            spread
        }
    }

    pub fn push(&mut self, new: f32) {
        self.history[self.write_idx] = new;
        self.write_idx = (self.write_idx + 1) % MAV_LENGTH;
    }

    pub fn evaluate(&mut self) -> f32 {
        self.history.iter().sum::<f32>() / MAV_LENGTH as f32
    }
}
