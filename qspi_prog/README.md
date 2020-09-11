# "Write File to External Flash" Program

This is a program for writing files to an STM32's external quad-SPI Flash memory. It runs in RAM, without modifying the chip's internal Flash memory.

It works by accepting commands over a UART connection, and responding once they are done. All commands and responses end with `\r\n`. Supported commands:

* `W[D]@[A]\r\n`: Write [D] bytes to address [A]. Response: `RDY\r\n`. Send [D] bytes after the response is received.

* `R[D]@[A]\r\n`: Read [D] bytes starting at address [A]. Response: `[data]\r\n`.

The "Write" command does not currently save and restore existing data in the sectors which it erases. The "Read" command might contain `\r` and `\n` characters in the returned data, so the reader should count bytes instead of looking for `\r\n`. Currently, only the first 256MB can be read since the program uses "memory-mapped" mode to perform the reads.

The application's stack is placed in the data bus' tightly-coupled memory, along with its UART receive buffer. The UART receive interrupt is stored in the instruction bus' tightly-coupled memory.
