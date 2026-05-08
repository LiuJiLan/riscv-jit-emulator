/* a_01_10 / 03_rvc_arith linker script
 * 跟 a_01_3/02_arith_compressed 同形态 — 纯 .S, BARE M-mode, 无 main.c.
 * .text + .rodata + .data 钉 0x80000000 起.
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
