#![no_main]
#![no_std]

use cortex_m::peripheral::Peripherals;
use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;

use mikoto_bot::{
    hal::{i2c::I2c, prelude::*},
    pac, Led,
};

use vl53l1::reg;

#[entry]
fn main() -> ! {
    // The Stm32 peripherals
    let dp = pac::Peripherals::take().unwrap();
    // The Cortex-m peripherals
    let core = Peripherals::take().unwrap();

    // Constrain clock registers
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();

    let mut syscfg = dp.SYSCFG.constrain();

    let mut delay = dp.TIM5.delay_us(&clocks);

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    // Configure PA5 (LD2 - User led) as an output
    let mut led = Led::new(gpioa.pa5);
    // Get the SCL and SDA pins of the I2C bus
    let sda = gpiob.pb3.into_alternate_open_drain();
    let scl = gpiob.pb10.into_alternate_open_drain();

    let mut i2c = I2c::new(dp.I2C2, (scl, sda), 400.kHz(), &clocks);

    // // Create a delay abstraction based on SysTick
    let mut delay = core.SYST.delay(&clocks);

    let mut vl53l1_dev = vl53l1::Device::default();

    defmt::info!("Software reset...");
    while let Err(_e) = vl53l1::software_reset(&mut vl53l1_dev, &mut i2c, &mut delay) {
        defmt::info!("  Error");
        delay.delay_ms(100_u32);
    }
    defmt::info!("  Complete");

    defmt::info!("Data init...");
    while vl53l1::data_init(&mut vl53l1_dev, &mut i2c).is_err() {}
    defmt::info!("  Complete");

    defmt::info!("Static init...");
    while vl53l1::static_init(&mut vl53l1_dev).is_err() {}
    defmt::info!("  Complete");

    defmt::info!("Setting region of interest...");
    let roi = vl53l1::UserRoi {
        bot_right_x: 10,
        bot_right_y: 6,
        top_left_x: 6,
        top_left_y: 10,
    };
    while vl53l1::set_distance_mode(&mut vl53l1_dev, vl53l1::DistanceMode::Long).is_err() {}

    while vl53l1::set_user_roi(&mut vl53l1_dev, roi.clone()).is_err() {}
    defmt::info!("  Complete");

    defmt::info!("Setting timing budget and inter-measurement period...");
    while vl53l1::set_measurement_timing_budget_micro_seconds(&mut vl53l1_dev, 100_000).is_err() {}
    while vl53l1::set_inter_measurement_period_milli_seconds(&mut vl53l1_dev, 200).is_err() {}

    defmt::info!("Start measurement...");
    while vl53l1::start_measurement(&mut vl53l1_dev, &mut i2c).is_err() {}
    defmt::info!("  Complete");

    loop {
        defmt::info!("Wait measurement data ready...");
        if vl53l1::wait_measurement_data_ready(&mut vl53l1_dev, &mut i2c, &mut delay).is_err() {
            delay.delay_ms(100u32);
            continue;
        }

        match vl53l1::get_ranging_measurement_data(&mut vl53l1_dev, &mut i2c) {
            Err(_e) => {
                defmt::info!("  Error");
                delay.delay_ms(70u32);
            }
            Ok(rmd) => {
                defmt::info!("  {:#?} mm", rmd.range_milli_meter);
                if led.is_on() && rmd.range_milli_meter < 3000 {
                    led.toggle();
                }
                if !led.is_on() && rmd.range_milli_meter > 3000 {
                    led.toggle();
                }
                continue;
            }
        }

        while let Err(_e) =
            vl53l1::clear_interrupt_and_start_measurement(&mut vl53l1_dev, &mut i2c, &mut delay)
        {
            defmt::info!("  Error");
            delay.delay_ms(70u32);
        }
    }
}
