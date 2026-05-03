/* a01_3 / 02_arith_compressed linker script
 * 把 .text 钉在 0x80000000(GUEST_RAM_START)。
 * 单段, 单 PT_LOAD。.bin 用 objcopy 提取 raw bytes。
 * 与 01_arith_basic / a01_1 / 01_bin_basic 的 link.ld 完全一致 (fixture 自包含原则,
 * 暂不抽出共享 ld; 等真出现 3+ fixture 共用同 ld 时考虑提取)。
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
