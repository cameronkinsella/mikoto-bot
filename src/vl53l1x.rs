use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::blocking::i2c::{Write, WriteRead};

use vl53l1::Device;

/// VL53L1X time-of-flight sensor
pub struct Vl53l1x {
    device: Device,
}

impl Vl53l1x {
    pub fn new<I, E, D>(i2c: &mut I, delay: &mut D) -> Self
    where
        I: WriteRead<Error = E> + Write<Error = E>,
        D: DelayUs<u32> + DelayMs<u32>,
    {
        let mut vl53l1_dev = vl53l1::Device::default();

        defmt::debug!("Software reset...");
        while let Err(_e) = vl53l1::software_reset(&mut vl53l1_dev, i2c, delay) {
            defmt::warn!("  Error during soft reset");
            delay.delay_ms(100_u32);
        }
        defmt::debug!("  Complete");

        defmt::debug!("Data init...");
        while vl53l1::data_init(&mut vl53l1_dev, i2c).is_err() {}
        defmt::debug!("  Complete");

        defmt::debug!("Static init...");
        while vl53l1::static_init(&mut vl53l1_dev).is_err() {}
        defmt::debug!("  Complete");

        defmt::debug!("Setting region of interest...");
        let roi = vl53l1::UserRoi {
            bot_right_x: 10,
            bot_right_y: 6,
            top_left_x: 6,
            top_left_y: 10,
        };
        while vl53l1::set_distance_mode(&mut vl53l1_dev, vl53l1::DistanceMode::Long).is_err() {}

        while vl53l1::set_user_roi(&mut vl53l1_dev, roi).is_err() {}
        defmt::debug!("  Complete");

        defmt::debug!("Setting timing budget and inter-measurement period...");
        while vl53l1::set_measurement_timing_budget_micro_seconds(&mut vl53l1_dev, 100_000).is_err()
        {
        }
        while vl53l1::set_inter_measurement_period_milli_seconds(&mut vl53l1_dev, 200).is_err() {}

        defmt::debug!("Start measurement...");
        while vl53l1::start_measurement(&mut vl53l1_dev, i2c).is_err() {}
        defmt::debug!("  Complete");

        Self { device: vl53l1_dev }
    }

    pub fn read<I, E, D>(&mut self, i2c: &mut I, delay: &mut D) -> i16
    where
        I: WriteRead<Error = E> + Write<Error = E>,
        D: DelayUs<u32> + DelayMs<u32>,
    {
        loop {
            if vl53l1::wait_measurement_data_ready(&mut self.device, i2c, delay).is_err() {
                delay.delay_ms(1u32);
                continue;
            }

            match vl53l1::get_ranging_measurement_data(&mut self.device, i2c) {
                Err(_e) => {
                    defmt::warn!("  Error getting ranging measurement data");
                    delay.delay_ms(70u32);
                }
                Ok(rmd) => {
                    return rmd.range_milli_meter;
                }
            }

            while let Err(_e) =
                vl53l1::clear_interrupt_and_start_measurement(&mut self.device, i2c, delay)
            {
                defmt::info!("  Error clearing interrupt and restarting measurement");
                delay.delay_ms(70u32);
            }
        }
    }
}
