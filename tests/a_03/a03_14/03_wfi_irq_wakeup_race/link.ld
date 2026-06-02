/* a03_14 / 03_wfi_irq_wakeup_race linker script — 跟 a_03 fixture 同 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
