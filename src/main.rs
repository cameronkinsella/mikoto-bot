#![no_main]
#![no_std]

use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;

use lsm6dso::{GyroscopeFullScale, GyroscopeOutput, Lsm6dso};
use mikoto_bot::{
    hal::{i2c::I2c, prelude::*},
    pac,
};

const ADDR: u8 = 0x6B;

#[entry]
fn main() -> ! {
    // The Stm32 peripherals
    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();
    let mut delay = dp.TIM5.delay_us(&clocks);
    let mut _counter = dp.TIM2.counter_us(&clocks);

    let gpiob = dp.GPIOB.split();
    let scl = gpiob.pb8.into_alternate_open_drain();
    let sda = gpiob.pb9.into_alternate_open_drain();

    let i2c = I2c::new(dp.I2C1, (scl, sda), 100.kHz(), &clocks);

    let mut lsm = Lsm6dso::new(i2c, ADDR).unwrap();

    lsm.set_gyroscope_scale(GyroscopeFullScale::Dps125).unwrap();
    lsm.set_gyroscope_output(GyroscopeOutput::Rate12_5).unwrap();

    let mut vals: [(f32, f32, f32); 1500] = [lsm.read_gyro().unwrap(); 1500];
    loop {
        vals.fill_with(|| {
            delay.delay_ms(80u32);
            lsm.read_gyro().unwrap()
        });
        let mut x_sum: f64 = 0.0;
        let mut y_sum: f64 = 0.0;
        let mut z_sum: f64 = 0.0;
        for (x, y, z) in vals {
            x_sum += x as f64;
            y_sum += y as f64;
            z_sum += z as f64;
        }

        defmt::info!(
            "x: {}, y: {}, z: {}",
            x_sum / vals.len() as f64,
            y_sum / vals.len() as f64,
            z_sum / vals.len() as f64
        );
    }
}
