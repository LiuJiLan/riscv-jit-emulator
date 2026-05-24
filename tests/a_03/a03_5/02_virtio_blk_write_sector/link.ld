/* a03_5 / 02_virtio_blk_write_sector linker script — 跟 a_03 fixture 同 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
