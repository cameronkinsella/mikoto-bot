use crate::hal::{
    gpio::{Input, Output, Pin, PinMode, PushPull},
    prelude::*,
    timer::{CounterUs, Instance},
};
use crate::{distance_unit::*, Ultrasonic};
use core::{cmp::Ordering, fmt, marker::PhantomData};

/// URM37 v5.0 Ultrasonic sensor
pub struct Urm37<TIM: Instance, const P1: char, const N1: u8, const P2: char, const N2: u8> {
    trigger: Pin<P1, N1, Output<PushPull>>,
    echo: Pin<P2, N2, Input>,
    counter: CounterUs<TIM>,
}
impl<TIM: Instance, const P1: char, const N1: u8, const P2: char, const N2: u8>
    Ultrasonic<TIM, P1, N1, P2, N2> for Urm37<TIM, P1, N1, P2, N2>
{
    fn new(
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
}

impl<TIM: Instance, const P1: char, const N1: u8, const P2: char, const N2: u8>
    Urm37<TIM, P1, N1, P2, N2>
{
    pub fn read(&mut self) -> Option<Distance<PulseDuration>> {
        // 1 second timeout
        self.counter.start(1_000_000_u32.micros()).unwrap();

        // starting pulse
        self.trigger.set_high();
        Self::waste(&self.counter, 2);
        self.trigger.set_low();
        Self::waste(&self.counter, 10);
        self.trigger.set_high();
        // ending pulse

        // starting echo read
        Self::waste_until(&self.counter, |c| c.is_low(), &self.echo, 1_000_000)?;
        let pulse_duration =
            Self::waste_until(&self.counter, |c| c.is_high(), &self.echo, 1_000_000)?;

        self.counter.cancel().unwrap();
        let distance: Distance<PulseDuration> = Distance(pulse_duration, PhantomData);
        Some(distance)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Distance<U>(u32, PhantomData<U>);

impl<U: ValidUnit> Distance<U> {
    pub fn value(&self) -> u32 {
        self.0
    }
    pub fn new(value: u32) -> Distance<U> {
        Distance(value, PhantomData)
    }
}

impl Distance<PulseDuration> {
    // 50us = 1 cm (according to data sheet)
    pub fn as_cm(&self) -> Distance<Cm> {
        Distance(self.0 / 50, PhantomData)
    }

    // 2.54 cm/in
    pub fn as_inch(&self) -> Distance<Inch> {
        Distance(self.0 / 127, PhantomData)
    }
}

impl Distance<Cm> {
    pub fn as_pulse(&self) -> Distance<PulseDuration> {
        Distance(self.0 * 50, PhantomData)
    }
}

impl Distance<Inch> {
    pub fn as_pulse(&self) -> Distance<PulseDuration> {
        Distance(self.0 * 127, PhantomData)
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
