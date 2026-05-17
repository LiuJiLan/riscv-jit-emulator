/* a02_4 / 04_sip_view_consistent linker script — 同 01_mip_mie_basic */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
