#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = mikoto_bot::pac, peripherals = true)]
mod app {
    use core::fmt::Write;
    use mikoto_bot::{
        hal::{gpio::Edge, prelude::*},
        pac, Button, Led, Usart,
    };

    #[shared]
    struct Resources {
        button: Button,
        led: Led,
        usart: Usart<pac::USART2>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(ctx: init::Context) -> (Resources, Local, init::Monotonics) {
        // Device specific peripherals
        let mut dp = ctx.device;

        // Setup the system clock
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

        let mut syscfg = dp.SYSCFG.constrain();

        let gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();

        // Setup Button and enable external interrupt
        let mut button = Button::new(gpioc.pc13);
        button.enable_interrupt(Edge::Rising, &mut syscfg, &mut dp.EXTI);

        // Setup the led
        let led = Led::new(gpioa.pa5);

        // Configure sender for USART2
        let mut usart = Usart::new(gpioa.pa2, dp.USART2, &clocks);
        writeln!(usart.tx(), "init done").unwrap();

        (
            Resources { led, button, usart },
            Local {},
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        // The idle loop
        loop {}
    }

    #[task(binds = EXTI15_10, shared = [led, button, usart])]
    fn on_button_press(ctx: on_button_press::Context) {
        let mut led = ctx.shared.led;
        let mut button = ctx.shared.button;
        let mut usart = ctx.shared.usart;

        // Clear the interrupt
        button.lock(|b| b.clear_interrupt_pending_bit());

        // Toggle the led
        led.lock(|l: &mut Led| l.toggle());

        usart.lock(|u| writeln!(u.tx(), "Button pressed!").unwrap());
    }
}
