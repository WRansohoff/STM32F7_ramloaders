# Minimal "Run from RAM" STM32 Program

This is a minimal ephemeral STM32 program. It is designed to run from RAM without making any changes to the chip's non-volatile memory.

The program image must be loaded manually using a debugger, and it cannot persist across power cycles, but it also doesn't overwrite any of the chip's internal Flash memory.

The only tricky part of running a program from RAM is setting up the vector table. ARM Cortex-M chips use a `VTOR` system register to define where the vector table starts; by default, that value is set to `0x00000000`, which maps to the start of Flash memory at `0x08000000` as long as the `BOOT0` and `BOOT1` signals are set to zero.

You can set the `VTOR` register to any memory address, as long as it is properly aligned. So to create a RAM vector table, you can simply create an array of words with the `aligned(...)` GCC attribute, then set the `VTOR` register to hold that array's starting address. When you want to use an interrupt, you can set the appropriate entry in the array to the interrupt handler function's address.

This program is simple enough that it doesn't need interrupts, but to demonstrate how they work in RAM, I used the SysTick interrupt to keep track of time.

The linker script was also modified to remove the vector table section and place everything else in RAM.

The application's stack is placed in the data bus' tightly-coupled memory, and the SysTick interrupt is stored in the instruction bus' tightly-coupled memory.
