#![no_std]

pub use hal::pac;
pub use stm32f4xx_hal as hal;

mod led;
pub use led::Led;

mod button;
pub use button::Button;

mod usart;
pub use usart::Usart;

mod servo;
pub use servo::InputRange;
pub use servo::Servo;
pub use servo::ServoRanges;

mod ultrasonic;
pub use ultrasonic::unit;
pub use ultrasonic::Ultrasonic;

mod vl53l1x;
pub use vl53l1x::Vl53l1x;

pub mod hc_sr04;
pub use hc_sr04::HcSr04;

pub mod urm37;
pub use urm37::Urm37;

use hal::{
    gpio::{Alternate, Pin},
    prelude::*,
    rcc::Clocks,
    timer::Ch,
};
use pac::{TIM1, TIM3, TIM4, TIM5};
use stm32f4xx_hal::gpio::{PA11, PA8, PA9, PB8, PC6};

pub struct MikotoPeripherals {
    pub pb8: PB8,
    pub pa11: PA11,
    pub pc6: PC6,
    pub pa9: PA9,
    pub pa8: PA8,
    pub tim1: TIM1,
    pub tim3: TIM3,
    pub tim4: TIM4,
    pub tim5: TIM5,
}

pub struct Mikoto {
    front_wheel: Servo<TIM4, Pin<'B', 8, Alternate<2>>, Ch<2>>,
    left_wheel: Servo<TIM1, Pin<'A', 11, Alternate<1>>, Ch<3>>,
    right_wheel: Servo<TIM3, Pin<'C', 6, Alternate<2>>, Ch<0>>,
    pub ultrasonic: HcSr04<TIM5, 'A', 9, 'A', 8>,
}

pub enum Direction {
    Forward,
    Backward,
    Left,
    Right,
}

impl Direction {
    fn motor_direction(&self, speed: u32) -> (i32, i32, i32) {
        let speed = speed as i32;
        match *self {
            Self::Forward => (speed, speed, speed),
            Self::Backward => (-speed, -speed, -speed),
            Self::Left => (0, -speed, speed),
            Self::Right => (0, speed, -speed),
        }
    }
}

impl Mikoto {
    pub fn new(dp: MikotoPeripherals, clocks: &Clocks) -> Self {
        // Front wheel has 75% speed
        let mut front_wheel =
            Servo::new(750.0, 2250.0, dp.pb8.into_alternate(), dp.tim4, clocks).unwrap();
        let mut left_wheel =
            Servo::new(500.0, 2500.0, dp.pa11.into_alternate(), dp.tim1, clocks).unwrap();
        let mut right_wheel =
            Servo::new(500.0, 2500.0, dp.pc6.into_alternate(), dp.tim3, clocks).unwrap();

        front_wheel.set_input_range(InputRange::CONTINUOUS_RANGE.rev());
        left_wheel.set_input_range(InputRange::CONTINUOUS_RANGE);
        right_wheel.set_input_range(InputRange::CONTINUOUS_RANGE.rev());

        let counter = dp.tim5.counter_us(clocks);
        // Configure ultrasonic sensor
        let trig = dp.pa9;
        let echo = dp.pa8;
        let ultrasonic = HcSr04::new(trig, echo, counter);
        Self {
            front_wheel,
            left_wheel,
            right_wheel,
            ultrasonic,
        }
    }

    pub fn drive(&mut self, direction: Direction, speed: u32) -> Result<(), servo::Error> {
        let (front_speed, left_speed, right_speed) = direction.motor_direction(speed);

        self.front_wheel.set_position(front_speed)?;
        self.left_wheel.set_position(left_speed)?;
        self.right_wheel.set_position(right_speed)?;
        Ok(())
    }

    pub fn stop(&mut self) -> Result<(), servo::Error> {
        self.drive(Direction::Forward, 0)
    }
}
