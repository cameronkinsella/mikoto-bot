use crate::hal::{
    gpio::{Input, Output, Pin, PinMode, PushPull},
    prelude::*,
    timer::{CounterUs, Instance},
};
use core::{cmp::Ordering, fmt, marker::PhantomData};
use unit::*;

/// HC-SR04 Ultrasonic sensor
pub struct Ultrasonic<TIM: Instance, const P1: char, const N1: u8, const P2: char, const N2: u8> {
    trigger: Pin<P1, N1, Output<PushPull>>,
    echo: Pin<P2, N2, Input>,
    counter: CounterUs<TIM>,
}
impl<TIM: Instance, const P1: char, const N1: u8, const P2: char, const N2: u8>
    Ultrasonic<TIM, P1, N1, P2, N2>
{
    pub fn new(
        trigger: Pin<P1, N1, impl PinMode>,
        echo: Pin<P2, N2, impl PinMode>,
        counter: CounterUs<TIM>,
    ) -> Self {
        let trigger = trigger.into_push_pull_output();
        let echo = echo.into_pull_down_input();
        Self {
            trigger,
            echo,
            counter,
        }
    }

    pub fn read(&mut self) -> Option<Distance<PulseDuration>> {
        // 1 second timeout
        self.counter.start(1_000_000_u32.micros()).unwrap();

        // starting pulse
        self.trigger.set_low();
        self.waste(&self.counter, 2);
        self.trigger.set_high();
        self.waste(&self.counter, 10);
        self.trigger.set_low();
        // ending pulse

        // starting echo read
        self.waste_until(&self.counter, |c| c.is_high(), &self.echo, 1_000_000)?;
        let pulse_duration =
            self.waste_until(&self.counter, |c| c.is_low(), &self.echo, 1_000_000)?;

        self.counter.cancel().unwrap();
        let distance: Distance<PulseDuration> = Distance(pulse_duration, PhantomData);
        Some(distance)
    }

    fn waste(&self, c_us: &CounterUs<TIM>, us: u32) {
        let ts1 = c_us.now().ticks();
        while (c_us.now().ticks() - ts1) < us {}
    }

    fn waste_until<T>(
        &self,
        c_us: &CounterUs<TIM>,
        predicate: fn(_: &T) -> bool,
        dt: &T,
        us: u32,
    ) -> Option<u32> {
        let ts1 = c_us.now().ticks();
        while (c_us.now().ticks() - ts1) < us && !predicate(dt) {}

        if predicate(dt) {
            Some(c_us.now().ticks() - ts1)
        } else {
            None
        }
    }
}

/// Units to describe distance
pub mod unit {
    #[derive(Debug, Clone, Copy)]
    pub enum PulseDuration {}
    #[derive(Debug, Clone, Copy)]
    pub enum Cm {}
    #[derive(Debug, Clone, Copy)]
    pub enum Inch {}

    pub trait ValidUnit {
        const UNIT: &'static str;
    }
    impl ValidUnit for PulseDuration {
        const UNIT: &'static str = "us";
    }
    impl ValidUnit for Cm {
        const UNIT: &'static str = "cm";
    }
    impl ValidUnit for Inch {
        const UNIT: &'static str = "in";
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Distance<U>(u32, PhantomData<U>);

impl<U: ValidUnit> Distance<U> {
    pub fn value(&self) -> u32 {
        self.0
    }
}

impl Distance<PulseDuration> {
    pub fn new<U: ValidUnit>(value: u32) -> Distance<U> {
        Distance(value, PhantomData)
    }

    // Assuming speed of sound = 340 m/s.
    // 340 m/s * 100 cm/m * 10^-6 s/us = 0.034 cm/us.
    // Since the sound wave must reach the target then return,
    // distance (cm) = (pulse_duration * 0.034) / 2 = pulse_duration / 58
    pub fn as_cm(&self) -> Distance<Cm> {
        Distance(self.0 / 58, PhantomData)
    }

    // Using same process as explained for cm conversion:
    // distance (in) = pulse_duration / 148
    pub fn as_inch(&self) -> Distance<Inch> {
        Distance(self.0 / 148, PhantomData)
    }
}

impl Distance<Cm> {
    pub fn as_pulse(&self) -> Distance<PulseDuration> {
        Distance(self.0 * 58, PhantomData)
    }
}

impl Distance<Inch> {
    pub fn as_pulse(&self) -> Distance<PulseDuration> {
        Distance(self.0 * 148, PhantomData)
    }
}

impl<U: ValidUnit> fmt::Display for Distance<U> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{} {}", self.0, U::UNIT)
    }
}

impl<U: ValidUnit> defmt::Format for Distance<U> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "{} {}", self.0, U::UNIT);
    }
}

impl<U> PartialEq<Self> for Distance<U> {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }

    fn ne(&self, other: &Self) -> bool {
        self.0 != other.0
    }
}

impl<U> PartialOrd<Self> for Distance<U> {
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
