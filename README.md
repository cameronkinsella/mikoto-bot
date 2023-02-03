# Mikoto Bot

MTE 380 Wall Robot Embedded Software

### Hardware

Board: [Nucleo-f401re](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)

### Usage

#### Flash the main program

```cargo flash --chip stm32f401re```

Or, to see defmt outputs:

```DEFMT_LOG=debug cargo run```

#### Flash an example

```cargo flash --chip stm32f401re --example ultrasonic```

Or, to see defmt outputs:

```DEFMT_LOG=debug cargo run --example ultrasonic```

If probe fails to flash your board you probably need to update the firmware on the onboard programmer.
The updater can be found at: https://www.st.com/en/development-tools/stsw-link007.html

### Debugging

 * Use `st_nucleo_f4.cfg` if using OpenOCD.

### Board properties

 * User led on PA5
 * User button on PC13
 * Serial port through ST-LINK on USART2, Tx: PA2 and Rx: PA3

### External Sensors and Actuators

* TODO

This repository is based on https://github.com/jkristell/nucleo-f401re
