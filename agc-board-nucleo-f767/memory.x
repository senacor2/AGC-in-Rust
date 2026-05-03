/* STM32F767ZIT6 memory map (per DS11532 Rev 8 §5).
 *
 * FLASH: 2 MB at 0x0800_0000.
 * RAM:   512 KB contiguous at 0x2000_0000, made up of
 *          DTCM   128 KB @ 0x2000_0000
 *          SRAM1  368 KB @ 0x2002_0000
 *          SRAM2   16 KB @ 0x2007_C000
 *        Treated as one region here because the address ranges are
 *        contiguous and cortex-m-rt's default link.x expects a single RAM.
 *
 * Not declared here (consumed via dedicated linker sections in later phases):
 *   ITCM    16 KB  @ 0x0000_0000  — fast instruction RAM
 *   BKPSRAM  4 KB  @ 0x4002_4000  — battery-backed; will host the
 *                                   survives-RESTART subset of AgcState
 *                                   (see project memory
 *                                    project_battery_backed_bkpsram.md).
 */
MEMORY {
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
}
