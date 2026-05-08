/* a_01_9 / 02_m_s_trap_csrs linker script (跟 a01_9/01 同) */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
