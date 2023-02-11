#![no_main]
#![no_std]

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::Peripherals;
use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;

use mikoto_bot::{
    hal::{gpio::Edge, interrupt, prelude::*},
    pac, Button, InputRange, Led, Servo, ServoRanges,
};

// Used to signal to the main loop that it should toggle the led
static SIGNAL: AtomicBool = AtomicBool::new(false);

static BUTTON: Mutex<RefCell<Option<Button>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // The Stm32 peripherals
    let mut dp = pac::Peripherals::take().unwrap();
    // The Cortex-m peripherals
    let _core = Peripherals::take().unwrap();

    // Constrain clock registers
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();

    let mut syscfg = dp.SYSCFG.constrain();

    let mut delay = dp.TIM5.delay_us(&clocks);

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // Configure servo motor
    let mut servo =
        Servo::new(500.0, 2500.0, gpiob.pb5.into_alternate(), dp.TIM3, &clocks).unwrap();

    // Configure PA5 (LD2 - User led) as an output
    let mut led = Led::new(gpioa.pa5);

    // Configure PC5 (User B1) as an input and enable external interrupt
    let mut button = Button::new(gpioc.pc13);
    button.enable_interrupt(Edge::Rising, &mut syscfg, &mut dp.EXTI);

    cortex_m::interrupt::free(|cs| {
        BUTTON.borrow(cs).replace(Some(button));
    });

    // Optional: change the range of values we can use for inputs (default = 0-180)
    servo.set_input_range(InputRange::CONTINUOUS_RANGE);

    // Enable the external interrupt
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::EXTI15_10);
    }
    defmt::info!("init");
    defmt::info!("Start Position: {}", servo.position());
    loop {
        // Button (User B1) toggles state_change
        let state_change = SIGNAL.load(Ordering::SeqCst);
        if state_change {
            led.toggle();

            defmt::info!("Set position: -100");
            servo.set_position(-100).unwrap();
            defmt::info!("Current Position: {}", servo.position());
            delay.delay_ms(3000_u32);

            defmt::info!("Set position: 100");
            servo.set_position(100).unwrap();
            defmt::info!("Current Position: {}", servo.position());
            delay.delay_ms(3000_u32);

            defmt::info!("Set position: 0");
            servo.set_position(0).unwrap();
            defmt::info!("Current Position: {}", servo.position());
            delay.delay_ms(3000_u32);

            // set_position returns an error if you give it an invalid position (not within input range)
            defmt::info!("Set position: 999");
            servo
                .set_position(999)
                .unwrap_or_else(|e| defmt::warn!("Invalid position! Error: {}", e));
            defmt::info!("Current Position: {}", servo.position());
            delay.delay_ms(3000_u32);
        }
    }
}

#[interrupt]
fn EXTI15_10() {
    // Clear the interrupt
    cortex_m::interrupt::free(|cs| {
        let mut button = BUTTON.borrow(cs).borrow_mut();
        button.as_mut().unwrap().clear_interrupt_pending_bit();
    });
    let state_change = SIGNAL.load(Ordering::SeqCst);
    SIGNAL.store(!state_change, Ordering::SeqCst);
}
