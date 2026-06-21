/* b_03_1 / 06_smc_cross_page linker script
 * .text + .rodata + .data 钉 0x80000000 起 (跟 b_02 fixture 体例同).
 * fixture 不含 main.c (纯 stub.S), 无 .data / .rodata 需求.
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
