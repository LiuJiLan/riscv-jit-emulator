/* 04_elf_class64_reject linker script
 * 单 PT_LOAD @ 0x80000000。和 02 链接脚本一样,
 * 区别只在 Makefile 的 -march/-mabi 选 rv64 那套, 产出 ELFCLASS64。
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
