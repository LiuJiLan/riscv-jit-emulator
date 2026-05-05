/* a_01_7 / 01_trap_priv_save_restore linker script
 * .text 钉 0x80000000 (GUEST_RAM_START)
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
