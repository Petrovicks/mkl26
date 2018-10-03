MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 64K
    RAM  (rwx) : ORIGIN = 0x1FFFF800, LENGTH = 8K
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* You may want to use this variable to locate the call stack and static
   variables in different memory regions. Below is shown the default value */
/* _stack_start = ORIGIN(RAM) + LENGTH(RAM); */

EXTERN(_FLASHCONFIG);

SECTIONS
{
    .flashconfig 0x400 :
    {
        KEEP(*(.flashconfig))
        _stext = .;
    } > FLASH
}
