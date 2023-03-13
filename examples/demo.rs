#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = mikoto_bot::pac, peripherals = true)]
mod app {
    use mikoto_bot::pac::{I2C2, TIM2};
    use mikoto_bot::{
        hal::{
            gpio::{Alternate, Edge, OpenDrain, Pin},
            i2c::I2c,
            prelude::*,
            timer::DelayUs,
        },
        pac, Button, Direction, Led, Mikoto, MikotoPeripherals, MikotoWheels, Vl53l1x,
    };

    type I2c2 = I2c<
        I2C2,
        (
            Pin<'B', 10, Alternate<4, OpenDrain>>,
            Pin<'B', 3, Alternate<9, OpenDrain>>,
        ),
    >;

    #[shared]
    struct Resources {
        button: Button,
        led: Led,
    }

    #[local]
    struct Local {
        mikoto: Mikoto,
        delay: DelayUs<TIM2>,
        i2c: I2c<
            I2C2,
            (
                Pin<'B', 10, Alternate<4, OpenDrain>>,
                Pin<'B', 3, Alternate<9, OpenDrain>>,
            ),
        >,
    }

    #[init]
    fn init(ctx: init::Context) -> (Resources, Local, init::Monotonics) {
        // Device specific peripherals
        let mut dp: pac::Peripherals = ctx.device;

        // Setup the system clock
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

        let mut syscfg = dp.SYSCFG.constrain();

        let mut delay = dp.TIM2.delay_us(&clocks);

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();

        // Setup Button and enable external interrupt
        let mut button = Button::new(gpioc.pc13);
        button.enable_interrupt(Edge::Rising, &mut syscfg, &mut dp.EXTI);

        // Setup the led
        let led = Led::new(gpioa.pa5);

        // Get the SCL and SDA pins of the I2C bus
        let sda = gpiob.pb3.into_alternate_open_drain();
        let scl = gpiob.pb10.into_alternate_open_drain();

        let mut i2c = I2c::new(dp.I2C2, (scl, sda), 400.kHz(), &clocks);

        let mikoto_tof = Vl53l1x::new(&mut i2c, &mut delay);

        let mikoto_wheels = MikotoWheels {
            pb8: gpiob.pb8,
            pa11: gpioa.pa11,
            pc6: gpioc.pc6,
            tim1: dp.TIM1,
            tim3: dp.TIM3,
            tim4: dp.TIM4,
            tim5: dp.TIM5,
        };

        let mikoto_dp = MikotoPeripherals {
            tof: mikoto_tof,
            wheels: mikoto_wheels,
        };

        let mikoto = Mikoto::new(mikoto_dp, &clocks);
        (
            Resources { led, button },
            Local { mikoto, delay, i2c },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        // The idle loop
        loop {}
    }

    #[task(binds = EXTI15_10, shared = [led, button], local = [delay, mikoto, i2c])]
    fn on_button_press(ctx: on_button_press::Context) {
        let mut led = ctx.shared.led;
        let mut button = ctx.shared.button;
        let delay: &mut DelayUs<TIM2> = ctx.local.delay;
        let mikoto: &mut Mikoto = ctx.local.mikoto;
        let i2c: &mut I2c2 = ctx.local.i2c;

        // Clear the interrupt
        button.lock(|b: &mut Button| b.clear_interrupt_pending_bit());

        // Toggle the led
        led.lock(|l: &mut Led| l.toggle());

        mikoto.drive(Direction::Forward, 100).unwrap();

        loop {
            let current_distance = mikoto.tof.read(i2c, delay);
            if current_distance <= 1500 {
                break;
            }
            delay.delay_ms(100_u32);
        }

        mikoto.stop().unwrap();
    }
}
