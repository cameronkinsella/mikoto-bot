#![no_main]
#![no_std]

use cortex_m::peripheral::Peripherals;
use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;

use mikoto_bot::{hal::prelude::*, pac, urm37::Distance, Led, Ultrasonic, Urm37};

#[entry]
fn main() -> ! {
    // The Stm32 peripherals
    let dp = pac::Peripherals::take().unwrap();
    // The Cortex-m peripherals
    let _core = Peripherals::take().unwrap();

    // Constrain clock registers
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();

    let mut delay = dp.TIM5.delay_us(&clocks);
    let counter = dp.TIM2.counter_us(&clocks);

    let gpioa = dp.GPIOA.split();

    // Configure ultrasonic sensor
    let trig = gpioa.pa9;
    let echo = gpioa.pa8;
    let mut ultrasonic = Urm37::new(trig, echo, counter);

    // Configure PA5 (LD2 - User LED) as an output
    let mut led = Led::new(gpioa.pa5);

    // Distance at which the LED will turn on.
    // Units are cm (type Distance<Cm>), which is elided from the later comparison with the read distance.
    let led_distance = Distance::new(25_u32);

    defmt::info!("init");
    loop {
        let distance = ultrasonic.read().unwrap().as_cm();

        // Print distance
        defmt::info!("Distance: {}", distance);

        // turn the LED on if closer than 25cm, off if further than 25cm
        if (!led.is_on() && distance < led_distance) || (led.is_on() && distance >= led_distance) {
            led.toggle();
        }
        delay.delay_ms(300_u32);
    }
}
