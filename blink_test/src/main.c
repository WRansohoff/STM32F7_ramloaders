// Device header file.
#include "stm32f7xx.h"
// Standard library includes.
#include <string.h>

// Memory section boundaries which are defined in the linker script.
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _siitcm, _sidtcm, _sitcm, _sdtcm, _eitcm, _edtcm, _estack;

// RAM vector table
__attribute__((aligned(0x80)))
__attribute__((section(".dtcm_vars")))
volatile uint32_t irqs[ 120 ];

// Core system clock speed.
uint32_t SystemCoreClock = 16000000;
// Put the global "systick" counter in DTCM RAM.
__attribute__((section(".dtcm_vars")))
volatile uint32_t systick = 0;

// SysTick interrupt handler: increment the global 'systick' value.
__attribute__((section(".itcm_irqs")))
void systick_handler( void ) {
  ++systick;
}

// Error interrupt handler.
__attribute__((section(".itcm_irqs")))
void error_handler( void ) {
  // Infinite loop.
  while( 1 ) {};
}

// Simple blocking millisecond delay method.
void delay_ms( uint32_t ms ) {
  // Calculate the 'end of delay' tick value, then wait for it.
  uint32_t next = systick + ms;
  while ( systick < next ) { __WFI(); }
}

// Reset handler: set the stack pointer and branch to main().
__attribute__((naked)) __attribute__((section(".reset"))) void reset_handler( void ) {
  // Set the stack pointer to the 'end of stack' value.
  __asm__( "LDR r0, =_estack\n\t"
           "MOV sp, r0" );
  // Branch to main().
  __asm__( "B main" );
}

/**
 * Main program.
 */
int main( void ) {
  // Copy initialized data.
  memcpy( &_sdata, &_sidata, ( ( void* )&_edata - ( void* )&_sdata ) );
  memcpy( &_sitcm, &_siitcm, ( ( void* )&_eitcm - ( void* )&_sitcm ) );
  memcpy( &_sdtcm, &_sidtcm, ( ( void* )&_edtcm - ( void* )&_sdtcm ) );
  // Clear the .bss section.
  memset( &_sbss, 0x00, ( ( void* )&_ebss - ( void* )&_sbss ) );

  // Enable floating-point unit.
  SCB->CPACR    |=  ( 0xF << 20 );

  // Relocate the vector table.
  SCB->VTOR      =  ( uint32_t )&irqs;

  // Set the "end of stack" address.
  irqs[ 0 ] = _estack;
  // Set the RAM program's reset handler location.
  irqs[ 1 ] = ( uint32_t )reset_handler;
  // Set an entry for non-maskable and fault interrupts.
  irqs[ NonMaskableInt_IRQn + 16 ] = ( uint32_t )error_handler;
  irqs[ BusFault_IRQn + 16 ] = ( uint32_t )error_handler;
  irqs[ UsageFault_IRQn + 16 ] = ( uint32_t )error_handler;
  // Set an entry for the SysTick interupt.
  irqs[ SysTick_IRQn + 16 ] = ( uint32_t )systick_handler;
  // Setup the SysTick peripheral to generate 1ms ticks.
  SysTick_Config( SystemCoreClock / 1000 );

  // Enable GPIOA peripheral clock.
  RCC->AHB1ENR |=  ( RCC_AHB1ENR_GPIOAEN );
  // Configure PA5 as push-pull output.
  GPIOA->MODER |=  ( 1 << ( 5 * 2 ) );

  // Main loop: toggle the LED every half-second.
  while ( 1 ) {
    GPIOA->ODR ^=  ( 1 << 5 );
    delay_ms( 500 );
  }
}
