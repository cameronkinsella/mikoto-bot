#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = mikoto_bot::pac, peripherals = true)]
mod app {
    use mikoto_bot::hc_sr04::Distance;
    use mikoto_bot::unit::Cm;
    use mikoto_bot::{
        hal::{gpio::Edge, prelude::*},
        pac::{Peripherals, TIM2},
        Button, Direction, Led, Mikoto, MikotoPeripherals, Usart,
    };
    use stm32f4xx_hal::gpio::PB3;
    use stm32f4xx_hal::timer::DelayUs;

    #[shared]
    struct Resources {
        button: Button,
        led: Led,
    }

    #[local]
    struct Local {
        mikoto: Mikoto,
        delay: DelayUs<TIM2>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Resources, Local, init::Monotonics) {
        // Device specific peripherals
        let mut dp: Peripherals = ctx.device;

        // Setup the system clock
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

        let mut syscfg = dp.SYSCFG.constrain();

        let delay = dp.TIM2.delay_us(&clocks);

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();

        // Setup Button and enable external interrupt
        let mut button = Button::new(gpioc.pc13);
        button.enable_interrupt(Edge::Rising, &mut syscfg, &mut dp.EXTI);

        // Setup the led
        let led = Led::new(gpioa.pa5);

        let mikoto_dp = MikotoPeripherals {
            pb8: gpiob.pb8,
            pa11: gpioa.pa11,
            pc6: gpioc.pc6,
            pa9: gpioa.pa9,
            pa8: gpioa.pa8,
            tim1: dp.TIM1,
            tim3: dp.TIM3,
            tim4: dp.TIM4,
            tim5: dp.TIM5,
        };

        let mikoto = Mikoto::new(mikoto_dp, &clocks);
        (
            Resources { led, button },
            Local { mikoto, delay },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        // The idle loop
        loop {}
    }

    #[task(binds = EXTI15_10, shared = [led, button], local = [delay, mikoto])]
    fn on_button_press(ctx: on_button_press::Context) {
        let mut led = ctx.shared.led;
        let mut button = ctx.shared.button;
        let delay: &mut DelayUs<TIM2> = ctx.local.delay;
        let mikoto: &mut Mikoto = ctx.local.mikoto;

        // Clear the interrupt
        button.lock(|b: &mut Button| b.clear_interrupt_pending_bit());

        // Toggle the led
        led.lock(|l: &mut Led| l.toggle());

        mikoto.drive(Direction::Forward, 100).unwrap();

        loop {
            let current_distance = mikoto.ultrasonic.read().unwrap().as_cm();
            if current_distance <= Distance::new(20_u32) {
                break;
            }
            delay.delay_ms(100_u32);
        }
        mikoto.drive(Direction::Backward, 100).unwrap();
        loop {
            let current_distance = mikoto.ultrasonic.read().unwrap().as_cm();
            if current_distance >= Distance::new(50_u32) {
                break;
            }
            delay.delay_ms(100_u32);
        }

        mikoto.drive(Direction::Right, 100).unwrap();
        delay.delay_ms(5000_u32);
        mikoto.drive(Direction::Left, 100).unwrap();
        delay.delay_ms(5000_u32);
        mikoto.stop().unwrap();
    }
}
