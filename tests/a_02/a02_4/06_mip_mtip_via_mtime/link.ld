/* a02_4 / 06_mip_mtip_via_mtime linker script — 同 01_mip_mie_basic */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
