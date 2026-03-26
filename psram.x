/*
 * Supplementary linker script for PSRAM support on the Teensy 4.1.
 *
 * This script is emitted by build.rs when the `psram` feature is enabled.
 * It is automatically linked via `cargo:rustc-link-arg=-Tpsram.x`.
 *
 * Assumptions (see docs/psram-api-design.md "Assumptions and Fragility"):
 *
 * - t4link.x (from imxrt-rt RuntimeBuilder) defines a FLASH region or
 *   region alias. If imxrt-rt renames it, `AT> FLASH` below will fail.
 *
 * - t4link.x defines a .uninit output section. If imxrt-rt removes or
 *   renames it, `INSERT AFTER .uninit` will fail.
 *
 * - The linker supports INSERT AFTER (GNU ld does; verify for rust-lld).
 *
 * - MEMORY blocks are additive across -T scripts (standard GNU ld behavior).
 *
 * - PSRAM is memory-mapped at 0x70000000 after FlexSPI2 initialization.
 */

MEMORY {
    PSRAM (rwx) : ORIGIN = 0x70000000, LENGTH = 16M
}

SECTIONS {
    .psram.data : {
        __spsram_data = .;
        *(.psram.data .psram.data.*)
        __epsram_data = .;
    } > PSRAM AT> FLASH
} INSERT AFTER .uninit

__sipsram_data = LOADADDR(.psram.data);
