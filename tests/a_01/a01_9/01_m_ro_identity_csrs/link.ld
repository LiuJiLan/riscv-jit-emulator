/* a_01_9 / 01_m_ro_identity_csrs linker script
 * .text 钉 0x80000000 (跟 fixture 02/03 + a_01_8 04-09 同形态; 纯 .S 不需要 .data/.rodata)
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text : { *(.text) }
}
