

MEMORY
{
  ram 	(rwx) : ORIGIN = 0x20000000, LENGTH = 20K
  flash (rx)  : ORIGIN = 0x00000000, LENGTH = 128K
}

/* inital stack pointer at end of memory */
_estack =  0x20005000;

SECTIONS
  {
	.  = 0x00000000;          /* From 0x00000000 */
  .text : {
    *(.vectors)      /* Vector table */
    *(.text)        /* Program code */
    *(.rodata)      /* Read only data */
    *(.rodata*)      /* Read only data */
    _etext = .;
    } >flash
        .  = 0x20000000;   /* From 0x20000000 */      
  .data : {
    _data = .;
    *(.data)        /* Data memory */
    _edata = .;
    } >ram AT > flash
  .bss : {
    . = ALIGN(4);
    _bss = .;
    *(.bss)         /* Zero-filled run time allocate data memory */
    _ebss = .;
    } >ram AT > flash
 }  

/*  Libraries to link against. */

/*INPUT( -lstm32fw )*/

/*========== end of file ==========*/
