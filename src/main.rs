#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = mikoto_bot::pac, peripherals = true)]
mod app {
    use lazy_static::lazy_static;
    use mikoto_bot::angle_unit::{Degrees, Radians};
    use mikoto_bot::pac::{I2C1, I2C2, TIM2, TIM4};
    use mikoto_bot::{
        hal::{
            gpio::{Alternate, Edge, OpenDrain, Pin},
            i2c,
            i2c::I2c,
            prelude::*,
            timer::{CounterUs, DelayUs, Instance},
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
        ApproachWall,
        ClimbUp,
        ClimbOver,
        ClimbDown,
        FindPole,
        ApproachPole,
    }

    #[shared]
    struct Resources {
        button: Button,
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

        // Get the SCL and SDA pins of the I2C bus
        let sda1 = gpiob.pb9.into_alternate_open_drain();
        let scl1 = gpiob.pb8.into_alternate_open_drain();

        let sda2 = gpiob.pb3.into_alternate_open_drain();
        let scl2 = gpiob.pb10.into_alternate_open_drain();

        let i2c1 = I2c::new(dp.I2C1, (scl1, sda1), 400.kHz(), &clocks);
        let mut i2c2 = I2c::new(dp.I2C2, (scl2, sda2), 400.kHz(), &clocks);

        let mut gyro = Mpu6050::new(i2c1, &mut delay);
        gyro.calibrate(&mut counter);
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
            Resources { button, task },
            Local {
                mikoto,
                delay,
                counter,
                gyro,
                tof,
                i2c: i2c2,
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

        // Distance (in mm) from the wall in which we ignore any anomalies detected
        const BUFFER: f32 = 250.0;

        // Angle left and right the robot should scan for anomalies (max 90 deg)
        const SCAN_ANGLE: Angle<Degrees> = Angle::new(80.0);

        let mut c_started = false;

        let mut offset_angle: Angle<Degrees> = Angle::new(0.0);

        let mut on_pole_base = (false, false); // (pitch, roll)
        let mut stop_pole_base = false;
        let mut pole_zero_pitch = Angle::new(0.0);
        let mut pole_zero_roll = Angle::new(0.0);

        let mut scan_pause = false;

        enum Scan {
            Stop,
            Left,
            Right,
        }

        let mut scan = Scan::Stop;

        // The idle loop
        loop {
            let mut gyro_reading = gyro.read();
            // Gyro is mounted upside down, so directions are reversed.
            gyro_reading = YawPitchRoll::from(YPR {
                yaw: gyro_reading.yaw.value(),
                pitch: gyro_reading.pitch.value(),
                roll: gyro_reading.roll.value(),
            });
            defmt::debug!(
                "yaw: {}, pitch: {}, roll: {}",
                gyro_reading.yaw.to_degrees(),
                gyro_reading.pitch.to_degrees(),
                gyro_reading.roll.to_degrees()
            );

            task.lock(|t: &mut Task| match t {
                Task::WaitForButton => {
                    mikoto.stop().unwrap();
                }
                Task::ApproachWall => {
                    if gyro_reading.pitch.to_degrees() >= Angle::new(60.0) {
                        defmt::info!("Mounted wall...");
                        *t = Task::ClimbUp;
                    }
                    // Changing pitch affects yaw measurements,
                    // so only conduct yaw correction while on the floor.
                    if gyro_reading.pitch.to_degrees() <= Angle::new(5.0) {
                        mikoto
                            .drive_straight(
                                gyro_reading.yaw,
                                offset_angle.to_radians(),
                                Direction::Forward,
                                100,
                            )
                            .unwrap()
                    } else {
                        mikoto.drive(Direction::Forward, 100).unwrap();
                    }
                }
                Task::ClimbUp => {
                    mikoto.drive(Direction::Forward, 100).unwrap();

                    #[allow(clippy::collapsible_if)]
                    if gyro_reading.pitch.to_degrees() <= Angle::new(45.0) {
                        if wait_until(counter, &mut c_started, 500_000) {
                            defmt::info!("Reached peak of wall...");
                            *t = Task::ClimbOver;
                        }
                    }
                }
                Task::ClimbOver => {
                    if gyro_reading.pitch.to_degrees() <= Angle::new(-45.0) {
                        mikoto.drive(Direction::Forward, 15).unwrap();
                    } else {
                        mikoto.drive(Direction::Forward, 100).unwrap();
                    }

                    #[allow(clippy::collapsible_if)]
                    if gyro_reading.pitch.to_degrees() <= Angle::new(-75.0) {
                        if wait_until(counter, &mut c_started, 1_500_000) {
                            defmt::info!("Climbed over peak of wall...");
                            *t = Task::ClimbDown;
                        }
                    }
                }
                Task::ClimbDown => {
                    mikoto.drive(Direction::Forward, 100).unwrap();

                    #[allow(clippy::collapsible_if)]
                    if gyro_reading.pitch.to_degrees() >= Angle::new(-10.0) {
                        if wait_until(counter, &mut c_started, 800_000) {
                            defmt::info!("Dismounted wall...");
                            *t = Task::FindPole;
                        }
                    }
                }
                Task::FindPole => match scan {
                    Scan::Stop => {
                        mikoto.stop().unwrap();

                        if scan_pause {
                            if wait_until(counter, &mut c_started, 500_000) {
                                offset_angle = gyro_reading.yaw.to_degrees();
                                pole_zero_pitch = gyro_reading.pitch.to_degrees();
                                pole_zero_roll = gyro_reading.roll.to_degrees();
                                scan_pause = false;
                                defmt::info!("Angle: {}", offset_angle);
                                *t = Task::ApproachPole;
                            }
                        } else {
                            scan = Scan::Left;
                        }
                    }
                    Scan::Left => {
                        let angle = gyro_reading.yaw;
                        let distance = tof.read(i2c, delay);
                        let expected = expected_dist(&angle);
                        mikoto.drive(Direction::Left, 5).unwrap();

                        if angle.to_degrees() <= -SCAN_ANGLE {
                            defmt::info!("Scanning right...");
                            scan = Scan::Right;
                        } else if distance as f32 <= expected - BUFFER {
                            defmt::info!("Pole detected!");
                            defmt::info!("Distance: {} mm", distance);
                            scan = Scan::Stop;
                            scan_pause = true;
                        }
                    }
                    Scan::Right => {
                        let angle = gyro_reading.yaw;
                        let distance = tof.read(i2c, delay);
                        let expected = expected_dist(&angle);
                        mikoto.drive(Direction::Right, 5).unwrap();

                        if angle.to_degrees() >= SCAN_ANGLE {
                            defmt::info!("Scanning left...");
                            scan = Scan::Left;
                        } else if distance as f32 <= expected - BUFFER {
                            defmt::info!("Pole detected!");
                            defmt::info!("Distance: {} mm", distance);
                            scan = Scan::Stop;
                            scan_pause = true;
                        }
                    }
                },
                Task::ApproachPole => {
                    let distance = tof.read(i2c, delay);
                    let found_pole = distance < 150;

                    // front wheel, left/right wheel
                    on_pole_base = (
                        gyro_reading.pitch.to_degrees() > Angle::new(pole_zero_pitch.value() + 2.0), // pitch
                        libm::fabsf(
                            gyro_reading.roll.to_degrees().value() - pole_zero_roll.value(),
                        ) >= 3.0, // roll
                    );

                    /*
                    Stop conditions:
                    0) Detect pole < 15cm away
                    1) Front wheel on pole base (pitch change +2 deg) for 250ms
                        THEN stays on pole base after stopping for 250ms
                    2) Left or right wheel on pole base (roll change +-3 deg)
                        THEN stays on pole base after stopping for 250ms
                    */
                    let stop_condition = (found_pole, on_pole_base.0, on_pole_base.1);

                    // Debounce pitch stop condition
                    #[allow(clippy::collapsible_if)]
                    if stop_condition.1 {
                        if wait_until(counter, &mut c_started, 250_000) {
                            stop_pole_base = true;
                        }
                    } else if !stop_condition.1 && c_started {
                        // False positive pitch condition: reset timer and do not stop
                        counter.cancel().unwrap();
                        c_started = false;
                    }

                    // Do not debounce roll stop condition
                    if stop_condition.2 {
                        stop_pole_base = true;
                    }

                    if stop_condition.0 || stop_pole_base {
                        mikoto.stop().unwrap();
                        if stop_condition.0 || wait_until(counter, &mut c_started, 250_000) {
                            if stop_condition.0 || stop_condition.1 || stop_condition.2 {
                                offset_angle = Angle::new(0.0);
                                defmt::info!("Pole found! Mission complete.");
                                *t = Task::WaitForButton;
                            }
                            stop_pole_base = false;
                        }
                    } else {
                        // False positive roll or pitch condition: resume driving
                        mikoto
                            .drive_straight(
                                gyro_reading.yaw,
                                offset_angle.to_radians(),
                                Direction::Forward,
                                100,
                            )
                            .unwrap();
                    }
                }
            });
        }
    }

    fn wait_until<TIM: Instance>(c_us: &mut CounterUs<TIM>, c_started: &mut bool, us: u32) -> bool {
        if !*c_started {
            c_us.start((2 * us).micros()).unwrap();
            *c_started = true;
            return false;
        }

        if c_us.now().ticks() > us {
            c_us.cancel().unwrap();
            *c_started = false;
            true
        } else {
            false
        }
    }

    // angle must be in radians
    fn expected_dist(angle: &Angle<Radians>) -> f32 {
        // All const distances in mm
        const TOF_DIST_FROM_BACK: f32 = 100.0;
        const COURSE_WIDTH: f32 = 2370.0;
        const COURSE_LENGTH: f32 = 2100.0;

        const RAMP_LENGTH: f32 = 1424.0;
        const RAMP_WIDTH: f32 = 318.0;

        lazy_static! {
            // All static angles in radians
            static ref CORNER_ANGLE: f32 = libm::atanf((COURSE_WIDTH / 2.0) / COURSE_LENGTH);
            static ref RAMP_ANGLE: f32 = 90_f32.to_radians()
                - libm::atanf(RAMP_LENGTH / ((COURSE_WIDTH / 2.0) - RAMP_WIDTH));
        }

        let adjacent_leg: f32;
        let mut theta = libm::fabsf(angle.value());
        if angle.value() > *RAMP_ANGLE {
            // Ramp is in line of sight
            theta = 90_f32.to_radians() - theta;
            adjacent_leg = (COURSE_WIDTH / 2.0) - RAMP_WIDTH;
        } else if theta > *CORNER_ANGLE {
            // Side border is in line of sight
            theta = 90_f32.to_radians() - theta;
            adjacent_leg = COURSE_WIDTH / 2.0;
        } else {
            // Rear border is in line of sight
            adjacent_leg = COURSE_LENGTH;
        }
        // adjacent / cos(theta) = hypotenuse
        (adjacent_leg / libm::cosf(theta)) - TOF_DIST_FROM_BACK
    }

    #[task(binds = EXTI15_10, shared = [button, task])]
    fn on_button_press(ctx: on_button_press::Context) {
        let mut button = ctx.shared.button;
        let mut task = ctx.shared.task;

        // Clear the interrupt
        button.lock(|b: &mut Button| b.clear_interrupt_pending_bit());

        defmt::info!("Button pressed!");
        task.lock(|t: &mut Task| *t = Task::ApproachWall);
    }
}
