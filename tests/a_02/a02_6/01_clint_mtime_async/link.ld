/* a02_6 / 01_clint_mtime_async linker script (0x80000000 .text, 跟 a02_5 同) */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
