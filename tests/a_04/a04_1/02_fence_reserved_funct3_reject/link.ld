/* a04_1 / 02_fence_reserved_funct3_reject linker script — 同 a_02 / a_03 fixture (0x80000000 .text) */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
