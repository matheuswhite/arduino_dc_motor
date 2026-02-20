#![no_std]
#![no_main]

use arduino_hal::simple_pwm::{IntoPwmPin, Prescaler, Timer2Pwm};
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    /*
     * For examples (and inspiration), head to
     *
     *     https://github.com/Rahix/avr-hal/tree/main/examples
     *
     * NOTE: Not all examples were ported to all boards!  There is a good chance though, that code
     * for a different board can be adapted for yours.  The Arduino Uno currently has the most
     * examples available.
     */

    let timer2 = Timer2Pwm::new(dp.TC2, Prescaler::Prescale64);
    let mut led = pins.d9.into_output().into_pwm(&timer2);
    
    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());
    let a0 = pins.a0.into_analog_input(&mut adc).into_channel();

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    let mut fade_amount = 5i32;
    let mut brightness = 0;

    loop {
        brightness += fade_amount;

        if brightness == 0 || brightness == 255 {
            // Reverse the direction of the fading at the ends of the fade:
            fade_amount = -fade_amount;
        }

        led.set_duty(brightness as u8);
        led.enable();

        let v = adc.read_blocking(&a0);

        let _ = ufmt::uwrite!(&mut serial, "{} {}\n", v, brightness);

        arduino_hal::delay_ms(30);
    }
}
