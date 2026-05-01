/* 02_elf_basic linker script
 * 单 PT_LOAD, .text 钉在 0x80000000。
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
