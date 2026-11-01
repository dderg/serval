use libm::{expf, roundf};

pub const HEATERS: usize = 2;
pub const DAC_FULL_SCALE: f32 = 4095.0;
pub const RATIO_MIN: f32 = 0.06;
pub const RATIO_MAX: f32 = 0.94;
const KELVIN: f32 = 273.15;
const SENSOR_REFERENCE_C: f32 = 25.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeaterParams {
    pub power_w: f32,
    pub capacity_j_per_k: f32,
    pub loss_w_per_k: f32,
    pub fan_loss_w_per_k: f32,
    pub ambient_c: f32,
    pub r25_ohm: f32,
    pub beta: f32,
    pub pullup_ohm: f32,
}

pub const EXTRUDER_DEFAULT: HeaterParams = HeaterParams {
    power_w: 40.0,
    capacity_j_per_k: 10.0,
    loss_w_per_k: 0.15,
    fan_loss_w_per_k: 0.15,
    ambient_c: 25.0,
    r25_ohm: 100_000.0,
    beta: 3000.0,
    pullup_ohm: 10_000.0,
};

pub const BED_DEFAULT: HeaterParams = HeaterParams {
    power_w: 200.0,
    capacity_j_per_k: 300.0,
    loss_w_per_k: 1.5,
    fan_loss_w_per_k: 0.0,
    ambient_c: 25.0,
    r25_ohm: 100_000.0,
    beta: 3000.0,
    pullup_ohm: 10_000.0,
};

impl HeaterParams {
    pub fn valid(&self) -> bool {
        let positive = [
            self.capacity_j_per_k,
            self.loss_w_per_k,
            self.r25_ohm,
            self.beta,
            self.pullup_ohm,
        ];
        let non_negative = [self.power_w, self.fan_loss_w_per_k];
        positive.iter().all(|v| v.is_finite() && *v > 0.0)
            && non_negative.iter().all(|v| v.is_finite() && *v >= 0.0)
            && self.ambient_c.is_finite()
    }

    pub fn adc_ratio(&self, temp_c: f32) -> f32 {
        let inverse = 1.0 / (temp_c + KELVIN) - 1.0 / (SENSOR_REFERENCE_C + KELVIN);
        let resistance = self.r25_ohm * expf(self.beta * inverse);
        resistance / (resistance + self.pullup_ohm)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Heater {
    pub params: HeaterParams,
    pub temp_c: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DacOutput {
    pub code: u16,
    pub clamped: bool,
}

impl Heater {
    pub fn at_ambient(params: HeaterParams) -> Self {
        Self {
            params,
            temp_c: params.ambient_c,
        }
    }

    pub fn step(&mut self, dt_s: f32, duty: f32, fan: f32) {
        let p = &self.params;
        let loss = p.loss_w_per_k + p.fan_loss_w_per_k * fan;
        let flow = p.power_w * duty - loss * (self.temp_c - p.ambient_c);
        self.temp_c += dt_s * flow / p.capacity_j_per_k;
    }

    pub fn dac(&self) -> DacOutput {
        let ratio = self.params.adc_ratio(self.temp_c);
        let clamped = !(RATIO_MIN..=RATIO_MAX).contains(&ratio);
        let ratio = ratio.clamp(RATIO_MIN, RATIO_MAX);
        DacOutput {
            code: (ratio * DAC_FULL_SCALE + 0.5) as u16,
            clamped,
        }
    }

    pub fn temp_centi(&self) -> i16 {
        roundf(self.temp_c * 100.0).clamp(i16::MIN as f32, i16::MAX as f32) as i16
    }
}
