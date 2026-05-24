/* a03_4 / 03_pending_bitmap_mmio_view linker script — 跟 a_02 / a_03 fixture 同 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
