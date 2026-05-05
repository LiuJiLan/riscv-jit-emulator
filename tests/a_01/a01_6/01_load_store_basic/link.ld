/* a_01_6 / 01_load_store_basic linker script
 * .text 钉 0x80000000 (GUEST_RAM_START)
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
