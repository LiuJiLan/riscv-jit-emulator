/* a02_4 / 05_mip_msip_via_clint linker script — 同 01_mip_mie_basic */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
