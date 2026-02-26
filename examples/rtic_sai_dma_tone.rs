//! SAI1 DMA tone generator for the Teensy Audio Shield (SGTL5000).
//!
//! Plays a continuous 440 Hz triangular-ish tone through the headphone
//! output of the Teensy Audio Shield (Rev D) using DMA-driven I2S via SAI1.
//!
//! This example verifies the Phase 0 HAL prerequisites:
//! - Audio PLL (PLL4) clocking SAI1
//! - SAI clock gates enabled
//! - SAI DMA peripheral traits (`Destination<u32>` for `Tx`)
//! - SGTL5000 basic initialisation via I2C
//!
//! Hardware: Teensy 4.1 + Audio Shield Rev D (SGTL5000)
//! Pins used:
//!   - p23: SAI1_MCLK
//!   - p26: SAI1_TX_BCLK
//!   - p27: SAI1_TX_SYNC (LRCLK)
//!   - p7:  SAI1_TX_DATA0
//!   - p20: SAI1_RX_SYNC
//!   - p21: SAI1_RX_BCLK
//!   - p8:  SAI1_RX_DATA0
//!   - p18: LPI2C1_SDA (SGTL5000 control)
//!   - p19: LPI2C1_SCL (SGTL5000 control)

#![no_std]
#![no_main]

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use bsp::board;
    use bsp::hal;
    use bsp::ral;
    use teensy4_bsp as bsp;

    use hal::dma::{
        channel::{self, Channel, Configuration},
        peripheral::Destination,
    };

    use embedded_hal::blocking::i2c::Write as I2cWrite;

    // ── Audio constants ──────────────────────────────────────────────

    /// Number of stereo samples per DMA buffer.
    const AUDIO_BLOCK_SAMPLES: usize = 128;

    /// DMA buffer length in u32 words (L + R interleaved).
    const DMA_BUF_LEN: usize = AUDIO_BLOCK_SAMPLES * 2;

    /// 256-entry waveform lookup table (triangle approximation of sine).
    ///
    /// Built at compile time with pure integer math (no floats, no libm).
    /// Amplitude ±32256 (Q15-ish). Sounds a bit buzzy but is perfectly
    /// adequate for verifying end-to-end audio output.
    static WAVE_TABLE: [i16; 256] = {
        let mut table = [0i16; 256];
        let mut i: usize = 0;
        while i < 256 {
            let phase = i as i32; // 0..255 ≙ 0..2π

            // quarter-wave index and sign
            let (idx, negate) = if phase < 64 {
                (phase, false)
            } else if phase < 128 {
                (128 - phase, false)
            } else if phase < 192 {
                (phase - 128, true)
            } else {
                (256 - phase, true)
            };
            // idx ∈ 0..64.  Triangle: ramp linearly
            let val: i16 = if idx <= 32 {
                (idx as i16) * 1008 // 0 → 32256
            } else {
                ((64 - idx) as i16) * 1008
            };
            table[i] = if negate { -val } else { val };
            i += 1;
        }
        table
    };

    // ── Type aliases ─────────────────────────────────────────────────

    type SaiTx = hal::sai::Tx<1, 32, 2, hal::sai::PackingNone>;

    // ── RTIC resources ───────────────────────────────────────────────

    #[local]
    struct Local {
        led: board::Led,
        dma_chan: Channel,
        sai_tx: SaiTx,
        phase: u32,
    }

    #[shared]
    struct Shared {}

    // ── Static DMA buffer ────────────────────────────────────────────
    // Place in OCRAM (.uninit region) which is non-cached and
    // DMA-accessible. Using .uninit means the section is NOLOAD so it
    // won't bloat the flash image / hex file. The buffer is filled
    // before every DMA transfer, so uninitialised contents are fine.
    #[link_section = ".uninit.dmabuffers"]
    static mut DMA_BUF: core::mem::MaybeUninit<[u32; DMA_BUF_LEN]> =
        core::mem::MaybeUninit::uninit();

    // ── Init ─────────────────────────────────────────────────────────

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio2,
            pins,
            mut dma,
            sai1,
            lpi2c1,
            ..
        } = board::t41(cx.device);

        // LED for heartbeat
        let led = board::led(&mut gpio2, pins.p13);

        // ── MCLK direction: output on pin 23 ────────────────────────
        unsafe {
            let gpr = ral::iomuxc_gpr::IOMUXC_GPR::instance();
            ral::modify_reg!(ral::iomuxc_gpr, gpr, GPR1, SAI1_MCLK_DIR: 1);
        }

        // ── Configure SAI1 ──────────────────────────────────────────
        // SAI must be configured and RX enabled BEFORE the SGTL5000 is
        // initialised — the codec needs MCLK present to respond on I2C.
        let sai = hal::sai::Sai::new(
            sai1,
            pins.p23, // MCLK
            hal::sai::Pins {
                sync: pins.p27, // TX_SYNC (LRCLK)
                bclk: pins.p26, // TX_BCLK
                data: pins.p7,  // TX_DATA0
            },
            hal::sai::Pins {
                sync: pins.p20, // RX_SYNC
                bclk: pins.p21, // RX_BCLK
                data: pins.p8,  // RX_DATA0
            },
        );

        let sai_config = {
            // bclk_div(4) → BCLK = MCLK/4 ≈ 2.8 MHz = 64×Fs
            // This matches Teensyduino (32-bit words, 64×Fs).
            let mut c = hal::sai::SaiConfig::i2s(hal::sai::bclk_div(4));
            c.sync_mode = hal::sai::SyncMode::TxFollowRx;
            // Select1 = MSEL 0b01 = MCLK1 (PLL4-derived, ~11.3 MHz).
            // The default Sysclk = MSEL 0b00 = IPG bus clock (150 MHz),
            // which is ~13x too fast.
            c.mclk_source = hal::sai::MclkSource::Select1;
            c
        };
        let (Some(mut sai_tx), Some(mut sai_rx)) =
            sai.split::<32, 2, hal::sai::PackingNone>(&sai_config)
        else {
            panic!("SAI split failed");
        };

        // Enable RX (clock master with TxFollowRx) so that BCLK and LRCLK
        // are present on the pins for the SGTL5000 during I2C init.
        sai_rx.set_enable(true);

        // ── I2C for SGTL5000 control ────────────────────────────────
        let mut i2c: board::Lpi2c1 = board::lpi2c(
            lpi2c1,
            pins.p19,
            pins.p18,
            board::Lpi2cClockSpeed::KHz400,
        );

        // ── Initialise SGTL5000 codec ───────────────────────────────
        sgtl5000_init(&mut i2c);

        // ── DMA channel 0 for SAI1 TX ───────────────────────────────
        let mut dma_chan = dma[0].take().expect("DMA channel 0");

        // Fill the initial tone buffer
        let phase = fill_tone_buffer(0);

        // Programme TCD: linear source buffer → hardware SAI TDR
        dma_chan.disable();
        dma_chan.set_disable_on_completion(true);
        dma_chan.set_interrupt_on_completion(true);
        dma_chan.set_channel_configuration(Configuration::enable(
            sai_tx.destination_signal(),
        ));
        unsafe {
            let buf = core::slice::from_raw_parts(
                core::ptr::addr_of!(DMA_BUF) as *const u32,
                DMA_BUF_LEN,
            );
            channel::set_source_linear_buffer(&mut dma_chan, buf);
            channel::set_destination_hardware(&mut dma_chan, sai_tx.destination_address());
            dma_chan.set_minor_loop_bytes(core::mem::size_of::<u32>() as u32);
            dma_chan.set_transfer_iterations(DMA_BUF_LEN as u16);
        }

        // Enable SAI TX DMA requests first, then start DMA, then enable
        // the transmitter. This order avoids a brief FIFO underrun at
        // startup.
        sai_tx.enable_dma_transmit();
        unsafe { dma_chan.enable() };
        sai_tx.set_enable(true);

        (
            Shared {},
            Local {
                led,
                dma_chan,
                sai_tx,
                phase,
            },
        )
    }

    // ── DMA channel 0 completion ISR ─────────────────────────────────

    #[task(binds = DMA0_DMA16, local = [led, dma_chan, sai_tx, phase, toggle: u32 = 0], priority = 2)]
    fn dma_complete(cx: dma_complete::Context) {
        let dma_chan = cx.local.dma_chan;
        let phase = cx.local.phase;
        let led = cx.local.led;
        let toggle = cx.local.toggle;

        // Acknowledge interrupt
        while dma_chan.is_interrupt() {
            dma_chan.clear_interrupt();
        }
        dma_chan.clear_complete();

        // Toggle LED as heartbeat (~1.3 Hz with 128-sample buffers @ 44 kHz)
        *toggle += 1;
        if *toggle % 172 == 0 {
            led.toggle();
        }

        // Fill buffer with next block of tone samples
        *phase = fill_tone_buffer(*phase);

        // Re-arm DMA
        unsafe {
            let buf = core::slice::from_raw_parts(
                core::ptr::addr_of!(DMA_BUF) as *const u32,
                DMA_BUF_LEN,
            );
            channel::set_source_linear_buffer(&mut *dma_chan, buf);
            dma_chan.set_transfer_iterations(DMA_BUF_LEN as u16);
            dma_chan.enable();
        }
    }

    // ── Helper: fill DMA buffer with 440 Hz tone ─────────────────────

    /// Phase-accumulator increment for 440 Hz at Fs ≈ 44 117.656 Hz.
    ///
    /// `440 * 65536 / 44118 ≈ 653`.  We use a Q16 accumulator whose top
    /// 8 bits index the 256-entry wave table.
    const PHASE_INC: u32 = 653;

    fn fill_tone_buffer(mut phase: u32) -> u32 {
        let buf = unsafe {
            core::slice::from_raw_parts_mut(
                core::ptr::addr_of_mut!(DMA_BUF) as *mut u32,
                DMA_BUF_LEN,
            )
        };
        for i in 0..AUDIO_BLOCK_SAMPLES {
            let idx = ((phase >> 8) & 0xFF) as usize;
            let sample = WAVE_TABLE[idx];
            // 16-bit sample → upper 16 bits of 32-bit I2S word (MSB-aligned).
            let sample32 = (sample as u16 as u32) << 16;

            // Stereo: identical sample on both channels
            buf[i * 2] = sample32;     // Left
            buf[i * 2 + 1] = sample32; // Right

            phase = phase.wrapping_add(PHASE_INC);
        }
        phase
    }

    // ── SGTL5000 I2C helpers ─────────────────────────────────────────

    fn sgtl5000_write<I2C: I2cWrite>(i2c: &mut I2C, reg: u16, val: u16)
    where
        I2C::Error: core::fmt::Debug,
    {
        let buf = [
            (reg >> 8) as u8,
            reg as u8,
            (val >> 8) as u8,
            val as u8,
        ];
        i2c.write(0x0A, &buf).unwrap();
    }

    fn sgtl5000_init<I2C: I2cWrite>(i2c: &mut I2C)
    where
        I2C::Error: core::fmt::Debug,
    {
        // ── Power-up phase ──────────────────────────────────────
        sgtl5000_write(i2c, 0x0030, 0x4060); // CHIP_ANA_POWER
        sgtl5000_write(i2c, 0x0026, 0x006C); // CHIP_LINREG_CTRL
        sgtl5000_write(i2c, 0x0028, 0x01F2); // CHIP_REF_CTRL
        sgtl5000_write(i2c, 0x002C, 0x0F22); // CHIP_LINE_OUT_CTRL
        sgtl5000_write(i2c, 0x003C, 0x4446); // CHIP_SHORT_CTRL
        sgtl5000_write(i2c, 0x0024, 0x0137); // CHIP_ANA_CTRL: mute all
        sgtl5000_write(i2c, 0x0030, 0x40FF); // CHIP_ANA_POWER: all on
        sgtl5000_write(i2c, 0x0002, 0x0073); // CHIP_DIG_POWER

        // Wait 400ms for VAG ramp & analog power-up (matches C++ delay(400)).
        cortex_m::asm::delay(240_000_000);

        // ── Volume & line-out ───────────────────────────────────
        sgtl5000_write(i2c, 0x002E, 0x1D1D); // CHIP_LINE_OUT_VOL

        // ── Clock & I2S configuration ───────────────────────────
        sgtl5000_write(i2c, 0x0004, 0x0004); // CHIP_CLK_CTRL: 44.1 kHz
        sgtl5000_write(i2c, 0x0006, 0x0030); // CHIP_I2S_CTRL: 16-bit I2S

        // ── Signal routing ──────────────────────────────────────
        sgtl5000_write(i2c, 0x000A, 0x0010); // CHIP_SSS_CTRL: I2S→DAC
        sgtl5000_write(i2c, 0x000E, 0x0000); // CHIP_ADCDAC_CTRL: unmute DAC
        sgtl5000_write(i2c, 0x0010, 0x3C3C); // CHIP_DAC_VOL: 0 dB
        sgtl5000_write(i2c, 0x0022, 0x1818); // CHIP_ANA_HP_CTRL: HP vol 0 dB
        sgtl5000_write(i2c, 0x0020, 0x0000); // CHIP_ANA_ADC_CTRL: ADC vol 0 dB

        // ── Unmute headphones ───────────────────────────────────
        // MUTE_LO=1, SELECT_HP=0(DAC), EN_ZCD_HP=1, MUTE_HP=0,
        // SELECT_ADC=1, EN_ZCD_ADC=1, MUTE_ADC=0
        sgtl5000_write(i2c, 0x0024, 0x0126); // CHIP_ANA_CTRL: unmute HP
    }
}
