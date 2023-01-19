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