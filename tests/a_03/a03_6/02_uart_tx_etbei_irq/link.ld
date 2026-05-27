/* a03_6 / 02_uart_tx_etbei_irq linker script — 跟 a_02 / a_03 fixture 同 (0x80000000 .text) */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
