use crate::angle_unit::*;
use crate::hal::prelude::*;
use core::f32::consts;
use core::{cmp::Ordering, fmt, marker::PhantomData, ops::Neg};
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::blocking::i2c::{Write, WriteRead};
use mpu6050_dmp::address::Address;
use mpu6050_dmp::quaternion::Quaternion;
use mpu6050_dmp::sensor;
use mpu6050_dmp::yaw_pitch_roll::YawPitchRoll as YPR;
use stm32f4xx_hal::timer::{CounterUs, Instance};

/// MPU-6050 3-axis gyroscope and a 3-axis accelerometer
pub struct Mpu6050<I, E>
where
    I: WriteRead<Error = E> + Write<Error = E>,
    E: core::fmt::Debug,
{
    device: sensor::Mpu6050<I>,
    offset: YawPitchRoll,
}

impl<I, E> Mpu6050<I, E>
where
    I: WriteRead<Error = E> + Write<Error = E>,
    E: core::fmt::Debug,
{
    pub fn new<D: DelayUs<u32> + DelayMs<u32>>(i2c: I, delay: &mut D) -> Self {
        let mut gyro = sensor::Mpu6050::new(i2c, Address::default()).unwrap();

        gyro.initialize_dmp(delay).unwrap();

        Self {
            device: gyro,
            offset: YawPitchRoll::from(YPR {
                yaw: 0.0,
                pitch: 0.0,
                roll: 0.0,
            }),
        }
    }

    pub fn calibrate<TIM: Instance>(&mut self, counter: &mut CounterUs<TIM>) {
        counter.start(21.secs()).unwrap();
        let wait_time = 20 * 1_000_000;
        let ts1 = counter.now().ticks();
        // Hold still and read values for 20 seconds to zero yaw-pitch-roll measurements
        while (counter.now().ticks() - ts1) < wait_time {
            defmt::info!("{}", 20 - (counter.now().ticks() - ts1) / (1000 * 1000));
            self.read();
        }
        let offset = self.read();
        self.offset = offset;
        defmt::info!("gyro initialized");
    }

    pub fn read(&mut self) -> YawPitchRoll {
        loop {
            let len = self.device.get_fifo_count().unwrap();
            if len >= 28 {
                let mut buf = [0; 28];
                let buf = self.device.read_fifo(&mut buf).unwrap();
                let quat = Quaternion::from_bytes(&buf[..16]).unwrap();
                let mut ypr = YPR::from(quat);
                ypr.yaw *= 2.0; // Sets range from 0 to +-180,
                Self::set_offset(&mut ypr.yaw, self.offset.yaw.value());
                Self::set_offset(&mut ypr.pitch, self.offset.pitch.value());
                Self::set_offset(&mut ypr.roll, self.offset.roll.value());

                return YawPitchRoll::from(ypr);
            }
        }
    }

    fn set_offset(value: &mut f32, offset: f32) {
        *value -= offset;
        if *value > consts::PI {
            *value -= 2.0 * consts::PI;
        } else if *value < -consts::PI {
            *value += 2.0 * consts::PI;
        }
    }
}

pub mod unit {
    #[derive(Debug, Clone, Copy)]
    pub enum Radians {}
    #[derive(Debug, Clone, Copy)]
    pub enum Degrees {}

    pub trait ValidUnit {
        const UNIT: &'static str;
    }
    impl ValidUnit for Radians {
        const UNIT: &'static str = " rad";
    }
    impl ValidUnit for Degrees {
        const UNIT: &'static str = "°";
    }
}

#[derive(Debug, Copy, Clone)]
pub struct YawPitchRoll {
    pub yaw: Angle<Radians>,
    pub pitch: Angle<Radians>,
    pub roll: Angle<Radians>,
}

impl From<YPR> for YawPitchRoll {
    fn from(value: YPR) -> Self {
        YawPitchRoll {
            yaw: Angle::new(value.yaw),
            pitch: Angle::new(value.pitch),
            roll: Angle::new(value.roll),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Angle<U>(f32, PhantomData<U>);

impl<U> Angle<U> {
    pub fn value(&self) -> f32 {
        self.0
    }
    pub const fn new(value: f32) -> Angle<U> {
        Angle(value, PhantomData)
    }
}

impl Angle<Radians> {
    pub fn to_degrees(&self) -> Angle<Degrees> {
        Angle(self.0.to_degrees(), PhantomData)
    }
}

impl Angle<Degrees> {
    pub fn to_radians(&self) -> Angle<Radians> {
        Angle(self.0.to_radians(), PhantomData)
    }
}

impl<U: ValidUnit> fmt::Display for Angle<U> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}{}", self.0, U::UNIT)
    }
}

impl<U: ValidUnit> defmt::Format for Angle<U> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "{}{}", self.0, U::UNIT);
    }
}

impl<U: ValidUnit> Neg for Angle<U> {
    type Output = Angle<U>;

    fn neg(self) -> Self::Output {
        Angle::new(-self.value())
    }
}

impl<U: ValidUnit> PartialEq<Self> for Angle<U> {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<U: ValidUnit> PartialOrd<Self> for Angle<U> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        if Self::eq(self, other) {
            return Some(Ordering::Equal);
        }
        if Self::lt(self, other) {
            return Some(Ordering::Less);
        }
        if Self::gt(self, other) {
            return Some(Ordering::Greater);
        }
        None
    }

    fn lt(&self, other: &Self) -> bool {
        self.0 < other.0
    }

    fn le(&self, other: &Self) -> bool {
        self.0 <= other.0
    }

    fn gt(&self, other: &Self) -> bool {
        self.0 > other.0
    }

    fn ge(&self, other: &Self) -> bool {
        self.0 >= other.0
    }
}
