# Mikoto Bot

MTE 380 Wall Robot Embedded Software

## Hardware

Board: [Nucleo-F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)

### Board properties

* User led on PA5
* User button on PC13
* Serial port through ST-LINK on USART2, Tx: PA2 and Rx: PA3

### External Sensors and Actuators

* TODO

## Usage: Local Environment

**IMPORTANT**: If you want to use WSL, you will need to do some [additional setup](./WSL_README.md).

### Prerequisites

1. Install [rust](https://www.rust-lang.org/tools/install).

2. Install [probe-run](https://github.com/knurling-rs/probe-run).

3. Install [cargo-flash](https://github.com/probe-rs/probe-rs/tree/master/cargo-flash).

4. Add the cross compilation target: `rustup target add thumbv7em-none-eabihf`

   > If you're using a different board then your required target may be different.
   > [List of ARM Cortex targets](https://docs.rust-embedded.org/cortex-m-quickstart/cortex_m_quickstart/).

### Run the main binary

Flash the program:

```bash
cargo flash --chip stm32f401re
```

Run with defmt debug outputs:

```bash
DEFMT_LOG=debug cargo run
```

### Run an example

Flash the program:

```bash
cargo flash --chip stm32f401re --example button-rtic
```

Run with defmt debug outputs:

```bash
DEFMT_LOG=debug cargo run --example button-interrupt
```

If probe fails to flash your board you probably need to update the firmware on the onboard programmer.
The updater can be found at: https://www.st.com/en/development-tools/stsw-link007.html

## Usage: Docker (Linux only)

This method also works in WSL if you have done the [additional setup](./WSL_README.md) for it.

### Find the path to your board's USB device

On Ubuntu (WSL):

```bash
export DEVICE_PATH=$(lsusb | grep ST-LINK | awk '{ print "/dev/bus/usb/" $2 "/" substr($4, 1, length($2))}')
```

```bash
$ echo $DEVICE_PATH
/dev/bus/usb/001/002
```

### Run the main binary

Flash the program:

```bash
./mikoto-cargo $DEVICE_PATH flash --chip stm32f401re
```

Run with defmt debug outputs:

```bash
./mikoto-cargo $DEVICE_PATH run
```

### Run an example

Flash the program:

```bash
./mikoto-cargo $DEVICE_PATH flash --chip stm32f401re --example button-rtic
```

Run with defmt debug outputs:

```bash
./mikoto-cargo $DEVICE_PATH run --example button-interrupt
```

## Debugging

* Use `st_nucleo_f4.cfg` if using OpenOCD.

This repository is based on https://github.com/jkristell/nucleo-f401re
