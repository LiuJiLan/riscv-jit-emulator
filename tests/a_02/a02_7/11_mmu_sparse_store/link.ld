/* a02_7 / 11_mmu_sparse_store linker script — 0x80000000, 跟 a02_2/04 同模板 (.S+.c) */
OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) *(.text.*) }
    .rodata : { *(.rodata) *(.rodata.*) *(.srodata) *(.srodata.*) }
    .data : { *(.data) *(.data.*) *(.sdata) *(.sdata.*) }
}
