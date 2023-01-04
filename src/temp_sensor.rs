use esp32c3_hal as hal;
use hal::{
    adc::{AdcPin, ADC, ADC1},
    gpio::{Analog, Bank0GpioRegisterAccess, GpioPin, InputOutputAnalogPinType},
    prelude::*,
};

use core::cell::RefMut;
use libm::log;
use serde::Serialize;

static VCC: f64 = 3.3;
static R2: f64 = 10000.0;
static ADCMAX: f64 = 4095.0;

static A: f64 = 0.001129148;
static B: f64 = 0.000234125;
static C: f64 = 0.0000000876741;

#[derive(Serialize)]
pub struct Data {
    pub temperature: f64,
}

pub fn read_temp_send(
    mut adc1: RefMut<ADC<ADC1>>,
    mut pin: RefMut<AdcPin<GpioPin<Analog, Bank0GpioRegisterAccess, InputOutputAnalogPinType, 0>, ADC1>>,
) -> f64 {
    let pin_value: f64 = nb::block!(adc1.read(&mut pin)).unwrap();
    let v_out = pin_value as f64 * VCC / ADCMAX;
    let r = v_out * R2 / (VCC - v_out);

    let t_k = 1.0 / (A + (B * log(r)) + (C * log(r) * log(r) * log(r)));
    let t_c = t_k - 273.15;

    t_c
}
