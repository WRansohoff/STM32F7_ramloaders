#include "global.h"
#include "qspi.h"

// Memory section boundaries which are defined in the linker script.
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _siitcm, _sidtcm, _sitcm, _sdtcm, _eitcm, _edtcm, _estack;

// RAM vector table
__attribute__((aligned(0x200)))
__attribute__((section(".dtcm_vars")))
volatile uint32_t irqs[ 120 ];

// Core system clock speed.
uint32_t SystemCoreClock = 16000000;

// UART receive variables.
__attribute__((section(".dtcm_vars"))) volatile uint8_t rx_buf[ 256 ];
__attribute__((section(".dtcm_vars"))) volatile int rx_pos = 0;
__attribute__((section(".dtcm_vars"))) volatile int newline = 0;
__attribute__((section(".dtcm_vars"))) volatile int rx_prog = 0;

// USART6 interrrupt handler.
__attribute__((section(".itcm_irqs")))
void uart_rx( void ) {
  // 'Receive register not empty' interrupt.
  if ( USART6->ISR & USART_ISR_RXNE ) {
    // Copy new data into the buffer.
    char rx = USART6->RDR;
    if ( rx == '\n' ) { newline = 1; }
    if ( rx_pos < 256 ) {
      rx_buf[ rx_pos ] = rx;
      ++rx_pos;
    }
  }
}

// Error interrupt handler.
__attribute__((section(".itcm_irqs")))
void error_handler( void ) {
  // Infinite loop.
  while( 1 ) {};
}

// Override the '_write' clib method to implement 'printf' over UART.
int _write( int handle, char* data, int size ) {
  int count = size;
  while ( count-- ) {
    while ( !( USART6->ISR & USART_ISR_TXE ) ) {};
    USART6->TDR = *data++;
  }
  return size;
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

  // Set clock speed to 216MHz (each tick is a bit less than 5ns)
  // PLL out = ( 16MHz * ( N / M ) / P ). P = 2, N = 54, M = 2.
  //FLASH->ACR   |=  ( 7 << FLASH_ACR_LATENCY_Pos );
  RCC->PLLCFGR &= ~( RCC_PLLCFGR_PLLN |
                     RCC_PLLCFGR_PLLM );
  RCC->PLLCFGR |=  ( ( 54 << RCC_PLLCFGR_PLLN_Pos ) |
                     ( 2 << RCC_PLLCFGR_PLLM_Pos ) );
  RCC->CR      |=  ( RCC_CR_PLLON );
  while ( !( RCC->CR & RCC_CR_PLLRDY ) ) {};
  RCC->CFGR    |=  ( 2 << RCC_CFGR_SW_Pos );
  while ( ( RCC->CFGR & RCC_CFGR_SWS ) != ( 2 << RCC_CFGR_SWS_Pos ) ) {};
  SystemCoreClock = 216000000;

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
  // Set an entry for the USART6 interrupt.
  irqs[ USART6_IRQn + 16 ] = ( uint32_t )uart_rx;

  // Enable peripheral clocks: GPIOB-E, QSPI, USART6.
  RCC->AHB1ENR |=  ( RCC_AHB1ENR_GPIOBEN |
                     RCC_AHB1ENR_GPIOCEN |
                     RCC_AHB1ENR_GPIODEN |
                     RCC_AHB1ENR_GPIOEEN );
  RCC->AHB3ENR |=  ( RCC_AHB3ENR_QSPIEN );
  RCC->APB2ENR |=  ( RCC_APB2ENR_USART6EN );

  // Initialize pins C6 and C7 for USART6.
  GPIOC->MODER    |=  ( ( 2 << ( 6 * 2 ) ) |
                        ( 2 << ( 7 * 2 ) ) );
  GPIOC->OSPEEDR  |=  ( ( 2 << ( 6 * 2 ) ) |
                        ( 2 << ( 7 * 2 ) ) );
  GPIOC->AFR[ 0 ] |=  ( ( 8 << ( 6 * 4 ) ) |
                        ( 8 << ( 7 * 4 ) ) );
  // Initialize pins B2, B6, C9, C10, D13, E2 for QSPI.
  GPIOB->MODER    |=  ( ( 2 << ( 2 * 2 ) ) |
                        ( 2 << ( 6 * 2 ) ) );
  GPIOB->OSPEEDR  |=  ( ( 3 << ( 2 * 2 ) ) |
                        ( 3 << ( 6 * 2 ) ) );
  GPIOB->PUPDR    |=  ( 1 << ( 6 * 2 ) );
  GPIOB->AFR[ 0 ] |=  ( ( 9 << ( 2 * 4 ) ) |
                        ( 10 << ( 6 * 4 ) ) );
  GPIOC->MODER    |=  ( ( 2 << ( 9 * 2 ) ) |
                        ( 2 << ( 10 * 2 ) ) );
  GPIOC->OSPEEDR  |=  ( ( 3 << ( 9 * 2 ) ) |
                        ( 3 << ( 10 * 2 ) ) );
  GPIOC->AFR[ 1 ] |=  ( ( 9 << ( ( 9 - 8 ) * 4 ) ) |
                        ( 9 << ( ( 10 - 8 ) * 4 ) ) );
  GPIOD->MODER    |=  ( 2 << ( 13 * 2 ) );
  GPIOD->OSPEEDR  |=  ( 3 << ( 13 * 2 ) );
  GPIOD->AFR[ 1 ] |=  ( 9 << ( ( 13 - 8 ) * 4 ) );
  GPIOE->MODER    |=  ( 2 << ( 2 * 2 ) );
  GPIOE->OSPEEDR  |=  ( 3 << ( 2 * 2 ) );
  GPIOE->AFR[ 0 ] |=  ( 9 << ( 2 * 4 ) );

  // Setup USART6 for 115200-baud TX.
  USART6->BRR  =  ( SystemCoreClock / 115200 );
  USART6->CR1 |=  ( USART_CR1_UE |
                    USART_CR1_TE |
                    USART_CR1_RE |
                    USART_CR1_RXNEIE);

  // Setup the NVIC to enable interrupts.
  // Use 4 bits for 'priority' and 0 bits for 'subpriority'.
  NVIC_SetPriorityGrouping( 0 );
  // UART receive interrupts should be high priority.
  uint32_t uart_pri_encoding = NVIC_EncodePriority( 0, 1, 0 );
  NVIC_SetPriority( USART6_IRQn, uart_pri_encoding );
  NVIC_EnableIRQ( USART6_IRQn );

  // QSPI peripheral initialization.
  // Set Flash size; 512Mb = 64MB = 2^(25+1) bytes.
  QUADSPI->DCR |=  ( 25 << QUADSPI_DCR_FSIZE_Pos );
  // Set 1-wire data mode with 32-bit addressing.
  QUADSPI->CCR |=  ( ( 3 << QUADSPI_CCR_ADSIZE_Pos ) |
                     ( 1 << QUADSPI_CCR_IMODE_Pos ) );
  // Wait an extra half-cycle to read, and set a clock prescaler.
  QUADSPI->CR  |=  ( QUADSPI_CR_SSHIFT |
                     ( 2 << QUADSPI_CR_PRESCALER_Pos ) |
                     ( 3 << QUADSPI_CR_FTHRES_Pos ) );

  // Flash chip initialization.
  // Send 'enter QSPI mode' command.
  // Enable the peripheral.
  QUADSPI->CR  |=  ( QUADSPI_CR_EN );
  // Set the 'enter QSPI mode' instruction.
  QUADSPI->CCR |=  ( 0x35 << QUADSPI_CCR_INSTRUCTION_Pos );
  // Wait for the transaction to complete, and disable the peripheral.
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  QUADSPI->CR  &= ~( QUADSPI_CR_EN );
  // Wait for the 'QSPI mode enabled' bit.
  qspi_reg_wait( 0x05, 0x41, 0x40 );

  // Send 'enable 4-byte addressing' command.
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  QUADSPI->CR  &= ~( QUADSPI_CR_EN );
  QUADSPI->CCR &= ~( QUADSPI_CCR_INSTRUCTION );
  // Use all 4 data lines to send the instruction.
  QUADSPI->CCR |=  ( 3 << QUADSPI_CCR_IMODE_Pos );
  // Enable the peripheral and send the 'enable 4B addresses' command.
  QUADSPI->CR  |=  ( QUADSPI_CR_EN );
  QUADSPI->CCR |=  ( 0xB7 << QUADSPI_CCR_INSTRUCTION_Pos );
  // Wait for the transaction to complete, and disable the peripheral.
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  QUADSPI->CR  &= ~( QUADSPI_CR_EN );
  // Wait for the '4-byte addressing enabled' bit to be set.
  qspi_reg_wait( 0x15, 0x20, 0x20 );

  // Main loop: process commands and data.
  printf( "START\r\n" );
  while ( 1 ) {
    // Ready for a new command: print "RDY" and wait for a newline.
    printf( "RDY\r\n" );
    while ( !newline ) { __WFI(); }

    // Valid read or write commands start with 'R' or 'W'.
    if ( ( rx_buf[ 0 ] == 'W' ) || ( rx_buf[ 0 ] == 'R' ) ) {
      // Get the data length and starting address.
      // Values will be < 5,000,000,000.
      char dat_s[ 11 ] = { '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0' };
      char adr_s[ 11 ] = { '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0' };
      // 'Phase' variable. When an '@' character is encountered, it
      // moves from phase 0 (data length) to phase 1 (start address).
      int phase = 0;
      // Position in the destination C string.
      int pos = 0;
      // 'Invalid command' flag.
      int invalid = 0;

      // Iterate over the command string, extracting numeric values.
      for ( int i = 1; i < rx_pos; ++i ) {
        // 'Phase 0': Extract data length.
        if ( phase == 0 ) {
          // Move to phase 1 when an '@' character is reached.
          if ( rx_buf[ i ] == '@' ) {
            phase = 1;
            pos = 0;
          }
          else {
            // In ASCII, number values ('0' - '9') are 0x30 - 0x39.
            // Non-numeric parameters are invalid.
            if ( ( rx_buf[ i ] < 0x30 ) || ( rx_buf[ i ] > 0x39 ) ) {
              invalid = 1;
              break;
            }
            // Copy current character into the destination C string.
            dat_s[ pos ] = rx_buf[ i ];
            ++pos;
          }
        }
        // 'Phase 1': Extract start address.
        else {
          // '\r\n' marks the end of the command.
          if ( rx_buf[ i ] == '\r' ) { break; }
          // Copy the start address character if it is a number.
          else {
            if ( ( rx_buf[ i ] < 0x30 ) || ( rx_buf[ i ] > 0x39 ) ) {
              invalid = 1;
              break;
            }
            adr_s[ pos ] = rx_buf[ i ];
            ++pos;
          }
        }
      }

      // Print an error message if the command was invalid.
      if ( invalid ) {
        printf( "NOPE\r\n" );
      }
      // Process the read or write command.
      else {
        // Prepare values and convert strings to integers.
        rx_pos = 0;
        dat_s[ 10 ] = '\0';
        adr_s[ 10 ] = '\0';
        int dat_len = atoi( dat_s );
        int adr_pos = atoi( adr_s );
        // Commands with a data length of 0 bytes are invalid.
        if ( dat_len == 0 ) {
          printf( "NOPE0\r\n" );
        }
        // Process write command.
        else if ( rx_buf[ 0 ] == 'W' ) {
          // Receive and write data, one page at a time.
          // The last page which will be written to.
          int last_page = ( adr_pos + dat_len - 1 ) / 256;
          // Bytes remaining.
          int dat_left = dat_len;
          // Current write address.
          int cur_adr = adr_pos;
          // Address of the next page boundary.
          int next_page = ( cur_adr / 256 ) + 1;
          // Number of bytes to transfer in the current page write.
          int rx_len = dat_left;
          if ( ( cur_adr + dat_left ) > ( next_page * 256 ) ) {
            rx_len = ( next_page * 256 ) - cur_adr;
          }

          // Erase any sectors which contain data to be written.
          printf( "SE...\r\n" );
          for ( int i = ( cur_adr / 0x1000 );
                i < ( ( ( cur_adr + dat_len ) / 0x1000 ) + 1 );
                ++i ) {
            printf( "%d...\r\n", i );
            qspi_erase_sector( i );
          }

          // Receive one page of data at a time; print "RDY" when
          // the next page of data can be sent.
          for ( int i = cur_adr / 256; i < last_page + 1; ++i ) {
            printf( "RDY\r\n" );
            // Wait for the appropriate number of bytes.
            while ( rx_pos < rx_len ) { __WFI(); }
            rx_pos = 0;
            // Write the page of data.
            qspi_write_page( cur_adr, rx_len, ( uint8_t* )rx_buf );
            // Update bookkeeping values.
            cur_adr = next_page * 256;
            ++next_page;
            dat_left -= rx_len;
            if ( dat_left >= 256 ) { rx_len = 256; }
            else { rx_len = dat_left; }
          }

          // Done.
          printf( "OK\r\n" );
        }
        // Process read command.
        else if ( rx_buf[ 0 ] == 'R' ) {
          // Enable memory-mapped mode.
          QUADSPI->CR  &= ~( QUADSPI_CR_EN );
          QUADSPI->CCR &= ~( QUADSPI_CCR_INSTRUCTION );
          QUADSPI->CCR |=  ( 3 << QUADSPI_CCR_FMODE_Pos |
                             3 << QUADSPI_CCR_ADMODE_Pos |
                             3 << QUADSPI_CCR_DMODE_Pos |
                             3 << QUADSPI_CCR_IMODE_Pos |
                             0xEC << QUADSPI_CCR_INSTRUCTION_Pos |
                             6 << QUADSPI_CCR_DCYC_Pos );
          QUADSPI->CR  |=  ( QUADSPI_CR_EN );
          __asm( "NOP" );

          // Read data and print it out.
          uint8_t* qflash = ( uint8_t* )0x90000000;
          for ( int i = 0; i < dat_len; ++i ) {
            putchar( qflash[ i + adr_pos ] );
          }
          printf( "\r\n" );
          fflush( stdout );

          // Exit memory-mapped mode.
          QUADSPI->CR   &= ~( QUADSPI_CR_EN );
          QUADSPI->CCR  &= ~( QUADSPI_CCR_INSTRUCTION |
                              QUADSPI_CCR_FMODE |
                              QUADSPI_CCR_IMODE |
                              QUADSPI_CCR_DMODE |
                              QUADSPI_CCR_ADMODE |
                              QUADSPI_CCR_DCYC );
        }
      }

      // Done running the command; reset bookkeeping values.
      rx_pos = 0;
      newline = 0;
    }
    // Invalid commands return an error.
    else {
      // Print a response refusing the invalid command.
      printf( "NOPE: %s\r\n", rx_buf );
      rx_pos = 0;
      newline = 0;
    }
  }
  return 0;
}
