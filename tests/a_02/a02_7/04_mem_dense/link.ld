/* a02_7 / 04_mem_dense linker script — 0x80000000 .text (跟 a02_6 同模板) */
OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
