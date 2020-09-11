# "RAM-Loaded" STM32F7 Programs

A couple of examples demonstrating how to run an STM32 application in RAM without modifying its Flash memory, and how to use "Tightly-Coupled Memory" banks to speed up interrupts and frequently-accessed variables.

The target hardware is an STM32F723E Discovery Kit. It has an external QSPI Flash chip, and TCM RAM banks connected to its instruction and data buses.

This repository includes two RAM programs, see their README files for more details:

* `blink_test`: Blinks the board's orange LED to verify that the process of running a program from RAM works.

* `qspi_prog`: Initializes the board's USB/UART connection to listen for commands which program the connected QSPI Flash.

# Building and Running

Each example program has its own GCC Makefile, but I haven't written any scripts to load and run them. To do that, you can use openocd:

1. Connect to the board: `openocd -f stlink-v2-1.cfg -f stm32f7x.cfg`

2. In a different terminal window:

2a. Connect to the newly-opened OpenOCD connection: `telnet localhost 4444`

2b. Reset and halt the STM32: `reset halt`

2c. Load the compiled program image: `load_image /path/to/main.elf`

2d. Run the RAM program: `resume 0x20010000`

After the `resume` command, the RAM program should start running normally. And when you reset the board (or enter `reset` in the telnet OpenOCD connection), it should "forget" the RAM program and go back to running whatever is in its internal Flash memory.
