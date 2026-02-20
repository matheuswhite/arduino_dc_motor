#![no_std]
#![no_main]

use arduino_hal::{
    adc::Channel,
    hal::port::PH6,
    port::{mode::PwmOutput, Pin},
    simple_pwm::{IntoPwmPin, Prescaler, Timer2Pwm},
    Adc, Peripherals,
};
use aule::prelude::*;
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();

    closed_loop(dp, 1.0, 0.0, 0.0);
}

fn closed_loop(dp: Peripherals, kp: f64, ki: f64, kd: f64) -> ! {
    let pins = arduino_hal::pins!(dp);

    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());
    let a0 = pins.a0.into_analog_input(&mut adc).into_channel();
    let mut sensor = Sensor::new(adc, a0);

    let timer2 = Timer2Pwm::new(dp.TC2, Prescaler::Prescale64);
    let pwm_output = pins.d9.into_output().into_pwm(&timer2);
    let mut actuator = Actuator::new(pwm_output);

    let mut reference = Step::new(5.0);
    let mut pid = PID::new(kp, ki, kd);

    for time in EndlessTime::new(0.03) {
        let sensor_signal = time * sensor.as_block();
        let ref_signal = time * reference.as_block();
        let error = ref_signal - sensor_signal;

        let output = error * pid.as_block();
        let _ = output * actuator.as_block();

        arduino_hal::delay_ms(time.delta.dt().as_millis() as u32);
    }

    unreachable!();
}

pub struct Sensor {
    adc: Adc,
    channel: Channel,
}

impl Sensor {
    pub fn new(adc: Adc, channel: Channel) -> Self {
        Self { adc, channel }
    }
}

impl Block for Sensor {
    type Input = ();
    type Output = f64;

    fn output(&mut self, input: Signal<Self::Input>) -> Signal<Self::Output> {
        input.map(|_| self.adc.read_blocking(&self.channel) as f64)
    }
}

pub struct Actuator {
    pwm: Pin<PwmOutput<Timer2Pwm>, PH6>,
}

impl Actuator {
    pub fn new(pwm: Pin<PwmOutput<Timer2Pwm>, PH6>) -> Self {
        Self { pwm }
    }
}

impl Block for Actuator {
    type Input = f64;
    type Output = ();

    fn output(&mut self, input: Signal<Self::Input>) -> Signal<Self::Output> {
        let duty = input.value.clamp(0.0, 255.0) as u8;

        self.pwm.set_duty(duty);
        self.pwm.enable();

        input.map(|_| ())
    }
}
