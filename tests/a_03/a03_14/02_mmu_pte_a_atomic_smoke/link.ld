/* a03_14 / 02_mmu_pte_a_atomic_smoke linker script
 *
 * .text @ 0x80000000 (跟 a02_7/08 / a03_14/01 同); .rodata/.data/.bss 紧跟
 * (因 main.c setup_pt 用栈 + 可能 const 字符)。
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text   : { *(.text) }
    .rodata : { *(.rodata) }
    .data   : { *(.data) }
    .bss    : { *(.bss COMMON) }
}
