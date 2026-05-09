/* a_01_10 / 07_cross_page linker script
 * 跟 a_01_10/03_rvc_arith 同形态 — 纯 .S, BARE M-mode, 无 main.c.
 * .text 段钉 0x80000000; stub.S 内用 .org 0xFF8 把 page_end_block 钉在 0x80000FF8,
 * cross_page 自然接在 0x80001000 (page_end_block 占 8 字节, .org 0xFF8 + 8 = 0x1000).
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
