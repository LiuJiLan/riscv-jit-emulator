/* a_01_5_c / 05_triple_fault_halt linker script
 * .text 钉 0x80000000 (GUEST_RAM_START)
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
