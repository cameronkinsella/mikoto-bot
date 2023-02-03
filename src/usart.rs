use crate::hal::{
    gpio::NoPin,
    prelude::*,
    rcc::Clocks,
    serial::{Config, Instance, Pins, Tx},
};

/// USART
pub struct Usart<USART: Instance> {
    tx: Tx<USART>,
}

// On Nucleo-f401re: tx_pin = PA2
impl<USART: Instance> Usart<USART> {
    pub fn new<TX>(tx_pin: TX, usart: USART, clocks: &Clocks) -> Self
    where
        (TX, NoPin): Pins<USART>,
    {
        let tx = usart
            .tx(
                tx_pin,
                Config::default()
                    .baudrate(115200.bps())
                    .wordlength_8()
                    .parity_none(),
                clocks,
            )
            .unwrap();
        Self { tx }
    }

    // Write to usart: writeln!(usart.tx(), "{}", s).unwrap();
    pub fn tx(&mut self) -> &mut Tx<USART> {
        &mut self.tx
    }
}
