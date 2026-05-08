/* a_01_8 / 04_sv32_lw_sw_basic linker script
 * .text + .rodata + .data 钉 0x80000000 起 (a_01_8 fixture 验证 .S+.c 模板首启用).
 * .bss 不分配 (raw binary 不含 NOBITS, fixture 没全局未初始化变量); .stack 不分配
 * (stub.S 直接 lui sp, 0x80300 设 sp 到 RAM 内空闲区, 不进 link.ld).
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
