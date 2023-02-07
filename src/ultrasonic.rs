use crate::hal::{
    gpio::{Pin, PinMode},
    timer::{CounterUs, Instance},
};

pub trait Ultrasonic<TIM: Instance, const P1: char, const N1: u8, const P2: char, const N2: u8> {
    fn new(
        trigger: Pin<P1, N1, impl PinMode>,
        echo: Pin<P2, N2, impl PinMode>,
        counter: CounterUs<TIM>,
    ) -> Self;

    fn waste(c_us: &CounterUs<TIM>, us: u32) {
        let ts1 = c_us.now().ticks();
        while (c_us.now().ticks() - ts1) < us {}
    }

    fn waste_until<T>(
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
