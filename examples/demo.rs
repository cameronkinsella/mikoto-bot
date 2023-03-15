#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = mikoto_bot::pac, peripherals = true)]
mod app {
    use mikoto_bot::angle_unit::*;
    use mikoto_bot::pac::{I2C1, I2C2, TIM2, TIM4};
    use mikoto_bot::{
        hal::{
            gpio::{Alternate, Edge, OpenDrain, Pin},
            i2c,
            i2c::I2c,
            prelude::*,
            timer::{CounterUs, DelayUs},
        },
        pac, Angle, Button, Direction, Led, Mikoto, MikotoPeripherals, MikotoWheels, Mpu6050,
        Vl53l1x, YawPitchRoll,
    };
    use mpu6050_dmp::yaw_pitch_roll::YawPitchRoll as YPR;

    type I2c1 = I2c<
        I2C1,
        (
            Pin<'B', 8, Alternate<4, OpenDrain>>,
            Pin<'B', 9, Alternate<4, OpenDrain>>,
        ),
    >;

    type I2c2 = I2c<
        I2C2,
        (
            Pin<'B', 10, Alternate<4, OpenDrain>>,
            Pin<'B', 3, Alternate<9, OpenDrain>>,
        ),
    >;

    #[derive(Debug, Clone, Copy)]
    pub enum Task {
        WaitForButton,
        TofDrive,
        PreciseTurn,
        Drive,
    }

    #[shared]
    struct Resources {
        button: Button,
        led: Led,
        task: Task,
    }

    #[local]
    struct Local {
        mikoto: Mikoto,
        delay: DelayUs<TIM4>,
        counter: CounterUs<TIM2>,
        i2c: I2c2,
        gyro: Mpu6050<I2c1, i2c::Error>,
        tof: Vl53l1x,
        next_task: Task,
    }

    #[init]
    fn init(ctx: init::Context) -> (Resources, Local, init::Monotonics) {
        // Device specific peripherals
        let mut dp: pac::Peripherals = ctx.device;

        // Setup the system clock
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

        let mut syscfg = dp.SYSCFG.constrain();

        let mut delay = dp.TIM4.delay_us(&clocks);
        let mut counter = dp.TIM2.counter_us(&clocks);

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();

        // Setup Button and enable external interrupt
        let mut button = Button::new(gpioc.pc13);
        button.enable_interrupt(Edge::Rising, &mut syscfg, &mut dp.EXTI);

        // Setup the led
        let mut led = Led::new(gpioa.pa5);

        // Starting task
        let task = Task::WaitForButton;
        let next_task = Task::TofDrive;

        // Get the SCL and SDA pins of the I2C bus
        let sda1 = gpiob.pb9.into_alternate_open_drain();
        let scl1 = gpiob.pb8.into_alternate_open_drain();

        let sda2 = gpiob.pb3.into_alternate_open_drain();
        let scl2 = gpiob.pb10.into_alternate_open_drain();

        let i2c1 = I2c::new(dp.I2C1, (scl1, sda1), 400.kHz(), &clocks);
        let mut i2c2 = I2c::new(dp.I2C2, (scl2, sda2), 400.kHz(), &clocks);

        let mut gyro = Mpu6050::new(i2c1, &mut delay);
        match next_task {
            _ => {
                // Don't need to use the gyro for every task, so don't waste time calibrating
                gyro.calibrate(&mut counter);
            }
        }

        let tof = Vl53l1x::new(&mut i2c2, &mut delay);

        let mikoto_wheels = MikotoWheels {
            pa1: gpioa.pa1,
            pa11: gpioa.pa11,
            pc6: gpioc.pc6,
            tim1: dp.TIM1,
            tim3: dp.TIM3,
            tim5: dp.TIM5,
        };

        let mikoto_dp = MikotoPeripherals {
            wheels: mikoto_wheels,
        };

        let mikoto = Mikoto::new(mikoto_dp, &clocks);

        // Toggle the led
        led.toggle();
        defmt::info!("Init complete");
        (
            Resources { led, button, task },
            Local {
                mikoto,
                delay,
                counter,
                gyro,
                tof,
                i2c: i2c2,
                next_task,
            },
            init::Monotonics(),
        )
    }

    #[idle(shared = [task], local = [mikoto, gyro, tof, i2c, counter, delay])]
    fn idle(ctx: idle::Context) -> ! {
        let mut task = ctx.shared.task;

        let mikoto: &mut Mikoto = ctx.local.mikoto;
        let gyro: &mut Mpu6050<I2c1, i2c::Error> = ctx.local.gyro;
        let tof: &mut Vl53l1x = ctx.local.tof;
        let i2c: &mut I2c2 = ctx.local.i2c;
        let counter: &mut CounterUs<TIM2> = ctx.local.counter;
        let delay: &mut DelayUs<TIM4> = ctx.local.delay;

        enum TurnStep {
            Left,
            Right,
        }

        let mut turn = TurnStep::Left;
        let mut turn_direction = Direction::Forward;

        enum DistanceStep {
            First,
            Second,
            Third,
            Forth,
        }

        const TOF_DISTANCE_FROM_FRONT: i16 = 95;

        impl DistanceStep {
            pub fn target_dist(&self) -> i16 {
                match *self {
                    DistanceStep::First => 2000 + TOF_DISTANCE_FROM_FRONT,
                    DistanceStep::Second => 1500 + TOF_DISTANCE_FROM_FRONT,
                    DistanceStep::Third => 1000 + TOF_DISTANCE_FROM_FRONT,
                    DistanceStep::Forth => 500 + TOF_DISTANCE_FROM_FRONT,
                }
            }

            pub fn next(&mut self) {
                match *self {
                    DistanceStep::First => {
                        *self = DistanceStep::Second;
                    }
                    DistanceStep::Second => {
                        *self = DistanceStep::Third;
                    }
                    DistanceStep::Third => {
                        *self = DistanceStep::Forth;
                    }
                    DistanceStep::Forth => {
                        *self = DistanceStep::First;
                    }
                }
            }
        }

        let mut distance_step = DistanceStep::First;

        let mut c_started = false;
        let mut c_start_t = 0;

        let mut start_turn = true;

        // The idle loop
        loop {
            let mut gyro_reading = gyro.read();
            // Gyro is mounted upside down, so directions are reversed.
            gyro_reading = YawPitchRoll::from(YPR {
                yaw: gyro_reading.yaw.value() * -1.0,
                pitch: gyro_reading.pitch.value() * -1.0,
                roll: gyro_reading.roll.value() * -1.0,
            });
            defmt::debug!(
                "yaw: {}°, pitch: {}°, roll: {}°",
                gyro_reading.yaw.to_degrees(),
                gyro_reading.pitch.to_degrees(),
                gyro_reading.roll.to_degrees()
            );

            task.lock(|t: &mut Task| match t {
                Task::WaitForButton => {}
                Task::TofDrive => {
                    mikoto
                        .drive_straight(gyro_reading.yaw, Angle::new(0.0), Direction::Forward, 100)
                        .unwrap();
                    let current_distance = tof.read(i2c, delay);
                    if !c_started && current_distance <= distance_step.target_dist() {
                        mikoto.stop().unwrap();
                        distance_step.next();
                        *t = Task::WaitForButton;
                    }
                }
                Task::PreciseTurn => match turn {
                    TurnStep::Left => {
                        if start_turn {
                            mikoto.drive(Direction::Left, 5).unwrap();
                            start_turn = false;
                        }

                        let desired_angle = match turn_direction {
                            Direction::Forward => Angle::new(-90.0),
                            Direction::Left => Angle::new(-179.0),
                            _ => panic!("invalid turn direction"),
                        };

                        if !c_started && gyro_reading.yaw.to_degrees() <= desired_angle {
                            defmt::debug!("Left turn stop");
                            mikoto.stop().unwrap();
                            c_started = true;
                            counter.start(3.secs()).unwrap();
                            c_start_t = counter.now().ticks();
                        } else if c_started && (counter.now().ticks() - c_start_t) > 1_000_000 * 2 {
                            c_started = false;
                            counter.cancel().unwrap();
                            match turn_direction {
                                Direction::Forward => {
                                    turn_direction = Direction::Left;
                                    turn = TurnStep::Left;
                                }
                                Direction::Left => {
                                    turn_direction = Direction::Backward;
                                    turn = TurnStep::Right;
                                }
                                _ => {}
                            }
                            start_turn = true;
                        }
                    }

                    TurnStep::Right => {
                        if start_turn {
                            mikoto.drive(Direction::Right, 5).unwrap();
                            start_turn = false;
                        }
                        let desired_angle: Angle<Degrees> = match turn_direction {
                            Direction::Backward => Angle::new(-90.0),
                            Direction::Left => Angle::new(-1.0),
                            _ => panic!("invalid turn direction"),
                        };

                        if !c_started
                            && libm::fabsf(gyro_reading.yaw.to_degrees().value())
                                <= libm::fabsf(desired_angle.value())
                        {
                            defmt::debug!("Right turn stop");
                            mikoto.stop().unwrap();
                            c_started = true;
                            counter.start(3.secs()).unwrap();
                            c_start_t = counter.now().ticks();
                        } else if c_started && (counter.now().ticks() - c_start_t) > 1_000_000 * 2 {
                            mikoto.stop().unwrap();
                            c_started = false;
                            counter.cancel().unwrap();
                            match turn_direction {
                                Direction::Backward => {
                                    turn_direction = Direction::Left;
                                    turn = TurnStep::Right;
                                }
                                Direction::Left => {
                                    turn_direction = Direction::Forward;
                                    turn = TurnStep::Left;
                                    *t = Task::WaitForButton;
                                }
                                _ => {}
                            }
                            start_turn = true;
                        }
                    }
                },
                Task::Drive => {
                    mikoto
                        .drive_straight(gyro_reading.yaw, Angle::new(0.0), Direction::Forward, 100)
                        .unwrap();
                }
            });
        }
    }

    #[task(binds = EXTI15_10, local = [next_task], shared = [button, task])]
    fn on_button_press(ctx: on_button_press::Context) {
        let mut button = ctx.shared.button;
        let mut task = ctx.shared.task;

        let next_task: &mut Task = ctx.local.next_task;

        // Clear the interrupt
        button.lock(|b: &mut Button| b.clear_interrupt_pending_bit());

        task.lock(|t: &mut Task| *t = *next_task);
    }
}
