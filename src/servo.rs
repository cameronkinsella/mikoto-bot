use crate::hal::{
    prelude::*,
    rcc::Clocks,
    time::Hertz,
    timer::{
        pwm::Pins,
        Channel,
        Channel::{C1, C2, C3, C4},
        PwmExt, PwmHz,
    },
};
use defmt_rtt as _;
use panic_probe as _;

#[derive(Debug, Eq, PartialEq, Copy, Clone, defmt::Format)]
pub enum Error {
    /// No PWM channels
    PwmDisabled,
    /// Invalid servo position
    InvalidPosition,
}

/// Servo Motor
pub struct Servo<TIM, PINS, P>
where
    PINS: Pins<TIM, P>,
    TIM: PwmExt,
{
    pwm: PwmHz<TIM, P, PINS>,
    channel: Channel,
    input_range: InputRange,
    min_duty: f64,
    max_duty: f64,
}

impl<TIM, PINS, P> Servo<TIM, PINS, P>
where
    PINS: Pins<TIM, P>,
    TIM: PwmExt,
{
    pub fn new(
        min_pulse: f64,
        max_pulse: f64,
        pin: PINS,
        timer: TIM,
        clocks: &Clocks,
    ) -> Result<Self, Error> {
        let pwm = timer.pwm_hz(pin, 50.Hz(), clocks);
        let channel = Self::open_channel()?;

        let mut servo = Self {
            pwm,
            channel,
            input_range: InputRange::POSITIONAL_RANGE,
            min_duty: 0f64,
            max_duty: 0f64,
        };
        servo.set_pulse(min_pulse, max_pulse);

        Ok(servo)
    }

    fn open_channel() -> Result<Channel, Error> {
        let pin_channels = [PINS::C1, PINS::C2, PINS::C3, PINS::C4];
        let channel = [C1, C2, C3, C4].into_iter().enumerate().find_map(|(i, c)| {
            if pin_channels[i] {
                Some(c)
            } else {
                None
            }
        });
        match channel {
            None => Err(Error::PwmDisabled),
            Some(c) => Ok(c),
        }
    }

    /// Returns the servo's zero position duty cycle
    fn zero(&self) -> f64 {
        self.min_duty + (self.max_duty - self.min_duty) / 2_f64
    }

    /// Set the servo's position. Must give a position within the input range.
    pub fn set_position(&mut self, position: i32) -> Result<(), Error> {
        let (low, high);
        if self.input_range.0 < self.input_range.1 {
            low = self.input_range.0;
            high = self.input_range.1;
        } else {
            low = self.input_range.1;
            high = self.input_range.0;
        }
        if !(low..=high).contains(&position) {
            return Err(Error::InvalidPosition);
        }

        let duty_limit = self.pwm.get_max_duty() as f64;

        self.pwm.set_duty(
            self.channel,
            libm::round(duty_limit * self.position_as_duty(position)) as u16,
        );
        self.pwm.enable(self.channel);
        Ok(())
    }

    /// Get the servo's current position.
    pub fn position(&self) -> i32 {
        let duty_ratio = self.pwm.get_duty(self.channel) as f64 / self.pwm.get_max_duty() as f64;
        self.duty_as_position(duty_ratio)
    }

    fn duty_as_position(&self, duty: f64) -> i32 {
        let input_start = self.input_range.0 as f64;
        let input_end = self.input_range.1 as f64;
        let position = (input_start * (duty - self.max_duty) + input_end * (self.min_duty - duty))
            / (self.min_duty - self.max_duty);
        libm::round(position) as i32
    }

    /// Converts a position to its corresponding duty cycle using the configured input range
    fn position_as_duty(&self, position: i32) -> f64 {
        let input_start = self.input_range.0 as f64;
        let input_end = self.input_range.1 as f64;

        (position as f64 - input_start) / (input_end - input_start)
            * (self.max_duty - self.min_duty)
            + self.min_duty
    }

    /// Set a new range for servo position values. Default = 0-180.
    pub fn set_input_range(&mut self, input_range: InputRange) {
        self.input_range = input_range
    }

    /// Set a new pulse range for the servo. Resets the servo to zero position.
    pub fn set_pulse(&mut self, min_pulse: f64, max_pulse: f64) {
        // Duty Cycle = pulse_width / period
        // period = 1 / frequency
        let period = 1_f64 / (self.pwm.get_period().raw() as f64 * 1e-6);
        self.min_duty = min_pulse / period;
        self.max_duty = max_pulse / period;
        let duty_limit = self.pwm.get_max_duty() as f64;
        self.pwm
            .set_duty(self.channel, libm::round(duty_limit * self.zero()) as u16);
        self.pwm.enable(self.channel);
    }

    /// Configure non-standard period (not 50Hz). Resets the servo to zero position.
    pub fn set_period(&mut self, freq: Hertz) {
        let period = 1_f64 / (self.pwm.get_period().raw() as f64 * 1e-6);
        let min_pulse = self.min_duty * period;
        let max_pulse = self.max_duty * period;
        self.pwm.set_period(freq);
        // Set new min and max duty cycle ratios
        self.set_pulse(min_pulse, max_pulse);
    }
}

/// Input values mapped to the servo's lower and upper limits respectively
pub type InputRange = (i32, i32);

pub trait ServoRanges {
    /// Default range. Range for positional servos, in degrees.
    const POSITIONAL_RANGE: InputRange = (0, 180);
    /// Range for continuous servos. Magnitude corresponds to speed and sign indicates direction.
    const CONTINUOUS_RANGE: InputRange = (-100, 100);

    fn rev(self) -> Self;
}

impl ServoRanges for InputRange {
    fn rev(self) -> Self {
        (self.1, self.0)
    }
}
