/* a03_1 / 02_plic_access_fault linker script — 跟 a_02 fixture 同 (0x80000000 .text) */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
