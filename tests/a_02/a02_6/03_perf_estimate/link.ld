/* a02_6 / 03_perf_estimate linker script (0x80000000 .text, 跟 a02_5 / a02_6 同) */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
