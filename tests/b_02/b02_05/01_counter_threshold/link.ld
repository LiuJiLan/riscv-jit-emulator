/* b_02_5 / 01_counter_threshold linker script
 * 把 .text 钉在 0x80000000 (GUEST_RAM_START).
 * 跟 a_01_4/01_branch_loop link.ld 一致 (fixture 自包含原则).
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
