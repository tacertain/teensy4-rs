//! Detects and tests PSRAM on the Teensy 4.1.
//!
//! Initializes FlexSPI2 PSRAM, runs a full memory test, then
//! blinks the LED to indicate the result:
//!
//! - **LED off**: no PSRAM detected
//! - **Single flash**: one 8 MB PSRAM chip installed
//! - **Double flash**: two PSRAM chips installed (16 MB)
//! - **Panic**: memory test failed
//!
//! The LED stays solid while the memory test is running.

#![no_std]
#![no_main]

use teensy4_bsp as bsp;
use teensy4_panic as _;

use bsp::{board, hal::timer::Blocking};

const FLASH_MS: u32 = 150;
const SHORT_PAUSE_MS: u32 = 250;
const LONG_PAUSE_MS: u32 = 1000;

#[bsp::rt::entry]
fn main() -> ! {
    let board::Resources {
        pit,
        pins,
        mut gpio2,
        ..
    } = board::t41(board::instances());
    let led = board::led(&mut gpio2, pins.p13);
    let mut delay = Blocking::<_, { board::PERCLK_FREQUENCY }>::from_pit(pit.0);

    // Detect PSRAM
    let size_mb = unsafe { board::init_psram() };

    if size_mb == 0 {
        led.clear();
        loop {
            cortex_m::asm::wfi();
        }
    }

    let size_bytes = size_mb as usize * 1024 * 1024;
    let chips = if size_mb > 8 { 2u8 } else { 1u8 };

    // LED solid while testing
    led.set();
    assert!(board::validate_psram(size_bytes), "PSRAM memory test failed");

    // Blink pattern: `chips` flashes then long pause
    loop {
        for i in 0..chips {
            led.set();
            delay.block_ms(FLASH_MS);
            led.clear();
            if i + 1 < chips {
                delay.block_ms(SHORT_PAUSE_MS);
            }
        }
        delay.block_ms(LONG_PAUSE_MS);
    }
}
