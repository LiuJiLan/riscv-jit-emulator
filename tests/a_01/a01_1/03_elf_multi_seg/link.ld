/* 03_elf_multi_seg linker script
 * 显式 PHDRS 强制 .text / .data 各自独立 PT_LOAD。
 * .text @ 0x80000000, .data @ 0x80100000(1MB gap)。
 * 验证 loader 能正确遍历多个 PT_LOAD, 各自落在 p_paddr 处。
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

PHDRS {
    text PT_LOAD;
    data PT_LOAD;
}

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) } :text

    . = 0x80100000;
    .data : { *(.data) } :data
}
