/* a_01_4 / 01_branch_loop linker script
 * 把 .text 钉在 0x80000000(GUEST_RAM_START)。
 * 单段, 单 PT_LOAD。.bin 用 objcopy 提取 raw bytes。
 * 与 a01_1 / 01_bin_basic, a01_3 fixture 的 link.ld 一致 (当前各自一份保持
 * fixture 自包含原则; 等真出现 5+ 个 fixture 共用同一 ld 时再考虑提取)。
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
