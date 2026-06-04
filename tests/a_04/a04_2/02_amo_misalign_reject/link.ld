/* a04_2 / 02_amo_misalign_reject linker script — 同 a_02 / a_03 fixture (0x80000000 .text) */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
