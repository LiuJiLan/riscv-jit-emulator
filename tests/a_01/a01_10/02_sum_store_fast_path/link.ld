/* a_01_10 / 02_sum_store_fast_path linker script
 * 跟 fixture 01 同形态 — .text + .rodata + .data 钉 0x80000000 起.
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
