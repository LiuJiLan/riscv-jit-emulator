/* a_01_10 / 04_rvc_branch linker script — 跟 fixture 03 同形态 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) *(.text.*) }
    .rodata : { *(.rodata) *(.rodata.*) *(.srodata) *(.srodata.*) }
    .data : { *(.data) *(.data.*) *(.sdata) *(.sdata.*) }
}
