/* a02_5 / 01_int_m_timer_basic linker script — 跟 a02_4 同 (0x80000000 .text) */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
