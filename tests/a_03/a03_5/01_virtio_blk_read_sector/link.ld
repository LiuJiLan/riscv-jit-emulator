/* a03_5 / 01_virtio_blk_read_sector linker script — 跟 a_02 / a_03 fixture 同 (0x80000000 .text) */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
