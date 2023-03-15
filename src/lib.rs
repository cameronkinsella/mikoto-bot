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
pub use ultrasonic::unit as distance_unit;
pub use ultrasonic::Ultrasonic;

mod vl53l1x;
pub use vl53l1x::Vl53l1x;

mod mpu6050;
pub use mpu6050::unit as angle_unit;
pub use mpu6050::Angle;
pub use mpu6050::Mpu6050;
pub use mpu6050::YawPitchRoll;

pub mod hc_sr04;
pub use hc_sr04::HcSr04;

pub mod urm37;
pub use urm37::Urm37;

use hal::{
    gpio::{Alternate, Pin},
    rcc::Clocks,
    timer::Ch,
};
use pac::{TIM1, TIM3, TIM5};
use stm32f4xx_hal::gpio::{PA1, PA11, PC6};

pub struct MikotoWheels {
    pub pa1: PA1,
    pub pa11: PA11,
    pub pc6: PC6,
    pub tim1: TIM1,
    pub tim3: TIM3,
    pub tim5: TIM5,
}

pub struct MikotoPeripherals {
    pub wheels: MikotoWheels,
}

pub struct Mikoto {
    front_wheel: Servo<TIM3, Pin<'C', 6, Alternate<2>>, Ch<0>>,
    left_wheel: Servo<TIM1, Pin<'A', 11, Alternate<1>>, Ch<3>>,
    right_wheel: Servo<TIM5, Pin<'A', 1, Alternate<2>>, Ch<1>>,
}

#[derive(Debug, Clone, Copy)]
pub enum Direction {
    Forward,
    Backward,
    Left,
    Right,
}

#[derive(Debug, Clone, Copy)]
pub enum VeerDirection {
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
        let mut front_wheel = Servo::new(
            750.0,
            2250.0,
            dp.wheels.pc6.into_alternate(),
            dp.wheels.tim3,
            clocks,
        )
        .unwrap();
        let mut left_wheel = Servo::new(
            500.0,
            2500.0,
            dp.wheels.pa11.into_alternate(),
            dp.wheels.tim1,
            clocks,
        )
        .unwrap();
        let mut right_wheel = Servo::new(
            500.0,
            2500.0,
            dp.wheels.pa1.into_alternate(),
            dp.wheels.tim5,
            clocks,
        )
        .unwrap();

        front_wheel.set_input_range(InputRange::CONTINUOUS_RANGE.rev());
        left_wheel.set_input_range(InputRange::CONTINUOUS_RANGE);
        right_wheel.set_input_range(InputRange::CONTINUOUS_RANGE.rev());

        Self {
            front_wheel,
            left_wheel,
            right_wheel,
        }
    }

    pub fn drive(&mut self, direction: Direction, speed: u32) -> Result<(), servo::Error> {
        let (front_speed, left_speed, right_speed) = direction.motor_direction(speed);

        self.front_wheel.set_position(front_speed)?;
        self.left_wheel.set_position(left_speed)?;
        self.right_wheel.set_position(right_speed)?;
        Ok(())
    }

    pub fn veer(&mut self, direction: VeerDirection, percentage: f32) -> Result<(), servo::Error> {
        if !(0.0..=100.0).contains(&percentage) {
            return Err(servo::Error::InvalidPosition);
        }

        let speed = libm::fmaxf(
            self.left_wheel.position() as f32,
            self.right_wheel.position() as f32,
        );

        let (new_left, new_right) = match direction {
            VeerDirection::Left => (((100.0 - percentage) / 100.0) * speed, speed),
            VeerDirection::Right => (speed, ((100.0 - percentage) / 100.0) * speed),
        };

        self.left_wheel.set_position(new_left as i32)?;
        self.right_wheel.set_position(new_right as i32)?;
        Ok(())
    }

    pub fn drive_straight(
        &mut self,
        current_yaw: Angle<angle_unit::Radians>,
        desired_angle: Angle<angle_unit::Radians>,
        direction: Direction,
        speed: u32,
    ) -> Result<(), servo::Error> {
        // Check that direction is front or back and set speed if not already set
        if let Direction::Forward = direction {
            if self.left_wheel.position() != speed as i32
                && self.right_wheel.position() != speed as i32
            {
                self.drive(direction, speed)?;
            }
        } else if let Direction::Backward = direction {
            if self.left_wheel.position() != -(speed as i32)
                && self.right_wheel.position() != -(speed as i32)
            {
                self.drive(direction, speed)?;
            }
        } else {
            return Err(servo::Error::InvalidPosition);
        }

        if current_yaw.value() - desired_angle.value() >= 2_f32.to_radians() {
            // offset right
            self.veer(VeerDirection::Left, 80.0)?;
        } else if current_yaw.value() - desired_angle.value() <= -2_f32.to_radians() {
            // offset left
            self.veer(VeerDirection::Right, 80.0)?;
        } else if libm::fabsf(current_yaw.value() - desired_angle.value()) <= 1_f32.to_radians() {
            // offset fixed
            self.drive(direction, speed)?;
        }
        Ok(())
    }

    pub fn stop(&mut self) -> Result<(), servo::Error> {
        self.drive(Direction::Forward, 0)
    }
}
