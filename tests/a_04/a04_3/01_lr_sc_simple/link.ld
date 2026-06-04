/* a04_3 / 01_lr_sc_simple linker script — 同 a04_2 fixture (0x80000000 .text) */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
