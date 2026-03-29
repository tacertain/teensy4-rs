//! Detects and tests PSRAM on the Teensy 4.1.
//!
//! Initializes FlexSPI2 PSRAM using the safe `initialize_psram` API,
//! then demonstrates accessing a typed PSRAM-backed buffer via
//! `psram_static!`. Blinks the LED to indicate the result:
//!
//! - **LED off**: no PSRAM detected
//! - **Single flash**: PSRAM initialized and buffer test passed
//! - **Panic**: buffer test failed

#![no_std]
#![no_main]

use teensy4_bsp as bsp;
use teensy4_panic as _;

use bsp::{board, hal::timer::Blocking, psram_static};

const FLASH_MS: u32 = 150;
const LONG_PAUSE_MS: u32 = 1000;

psram_static! {
    static BUFFER: [u8; 1024] = [0u8; 1024];
}

#[bsp::rt::entry]
fn main() -> ! {
    let board::Resources {
        pit,
        pins,
        mut gpio2,
        flexspi2,
        mut iomuxc,
        ..
    } = board::t41(board::instances());
    let led = board::led(&mut gpio2, pins.p13);
    let mut delay = Blocking::<_, { board::PERCLK_FREQUENCY }>::from_pit(pit.0);

    // Initialize PSRAM and copy .psram.data from flash
    let token = match board::initialize_psram(flexspi2, &mut iomuxc, Default::default()) {
        Ok(psram) => psram.token(),
        Err(_) => {
            led.clear();
            loop {
                cortex_m::asm::wfi();
            }
        }
    };

    // Take a one-shot &'static mut reference to the PSRAM buffer
    let buf: &'static mut [u8; 1024] = BUFFER.take(token).unwrap();

    // Verify the buffer was initialized to all zeros
    assert!(buf.iter().all(|&b| b == 0), "PSRAM buffer not zeroed");

    // Write a test pattern and verify
    for (i, byte) in buf.iter_mut().enumerate() {
        *byte = (i & 0xFF) as u8;
    }
    for (i, byte) in buf.iter().enumerate() {
        assert_eq!(*byte, (i & 0xFF) as u8, "PSRAM readback mismatch");
    }

    // Blink to indicate success
    loop {
        led.set();
        delay.block_ms(FLASH_MS);
        led.clear();
        delay.block_ms(LONG_PAUSE_MS);
    }
}
