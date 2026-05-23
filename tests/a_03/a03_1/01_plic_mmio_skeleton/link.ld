/* a03_1 / 01_plic_mmio_skeleton linker script — 跟 a_02 fixture 同 (0x80000000 .text) */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
