/* a02_4 / 01_mip_mie_basic linker script
 * 把 .text 钉在 0x80000000 (GUEST_RAM_START).
 * 单段, 单 PT_LOAD. .bin 用 objcopy 提取 raw bytes.
 * 与 a02_2 / a02_3 fixture 的 link.ld 一致 (fixture 自包含原则).
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
