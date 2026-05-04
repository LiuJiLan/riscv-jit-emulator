/* a_01_5_b / 02_trap_arch_basic linker script
 * 把 .text 钉在 0x80000000(GUEST_RAM_START)。
 * 单段, 单 PT_LOAD。.bin 用 objcopy 提取 raw bytes。
 * 与 a01_4 / 01_csr_basic fixture 的 link.ld 一致。
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
