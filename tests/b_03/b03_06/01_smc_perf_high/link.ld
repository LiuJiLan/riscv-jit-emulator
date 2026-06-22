/* b_03_6 / 01_smc_perf_high linker script
 * .text + .rodata + .data 钉 0x80000000 起 (跟 a02_7 perf 套件 / b_03 fixture 体例同).
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
