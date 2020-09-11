/* Label for the program's entry point */
ENTRY(reset_handler)

/* Starting stack address */
_estack = 0x20008000;

MEMORY
{
    RAM2      ( rxw )   : ORIGIN = 0x2003C000, LENGTH = 16K
    RAM       ( rxw )   : ORIGIN = 0x20010000, LENGTH = 176K
    DTCMRAM_S ( rxw )   : ORIGIN = 0x20000000, LENGTH = 32K
    DTCMRAM_V ( rxw )   : ORIGIN = 0x20008000, LENGTH = 32K
    ITCMRAM   ( rxw )   : ORIGIN = 0x00000000, LENGTH = 16K
}

INCLUDE "sections.ld"
