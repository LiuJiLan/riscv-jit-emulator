/* a_01_10 / 01_sum_load_fast_path linker script
 * 跟 a_01_8 fixture 04 模板同形态 — .text + .rodata + .data 钉 0x80000000 起.
 * .bss 不分配 (raw binary 不含 NOBITS); .stack 不分配 (stub.S 直接 lui sp 设栈).
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) *(.text.*) }
    .rodata : { *(.rodata) *(.rodata.*) *(.srodata) *(.srodata.*) }
    .data : { *(.data) *(.data.*) *(.sdata) *(.sdata.*) }
}
