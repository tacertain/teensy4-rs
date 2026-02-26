//! SAI1 polling tone generator for the Teensy Audio Shield (SGTL5000).
//!
//! Plays a continuous 440 Hz triangle tone through the headphone output
//! using polled FIFO writes — **no DMA involved**. This is a minimal
//! "does the audio board work at all?" sanity check.
//!
//! The SAI1 FIFO Request interrupt fires when the transmit FIFO drops
//! to or below the watermark.  The ISR writes one stereo frame at a
//! time until the FIFO is full.
//!
//! Hardware: Teensy 4.1 + Audio Shield Rev D (SGTL5000)
//! Pins:
//!   - p23  SAI1_MCLK
//!   - p26  SAI1_TX_BCLK
//!   - p27  SAI1_TX_SYNC (LRCLK)
//!   - p7   SAI1_TX_DATA0
//!   - p20  SAI1_RX_SYNC
//!   - p21  SAI1_RX_BCLK
//!   - p8   SAI1_RX_DATA0
//!   - p18  LPI2C1_SDA
//!   - p19  LPI2C1_SCL

#![no_std]
#![no_main]

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use bsp::board;
    use bsp::hal;
    use bsp::ral;
    use teensy4_bsp as bsp;

    use embedded_hal::blocking::i2c::Write as I2cWrite;

    // ── Audio constants ──────────────────────────────────────────────

    /// 256-entry triangle wave table.  Amplitude ±32256.
    static WAVE_TABLE: [i16; 256] = {
        let mut table = [0i16; 256];
        let mut i: usize = 0;
        while i < 256 {
            let phase = i as i32;
            let (idx, negate) = if phase < 64 {
                (phase, false)
            } else if phase < 128 {
                (128 - phase, false)
            } else if phase < 192 {
                (phase - 128, true)
            } else {
                (256 - phase, true)
            };
            let val: i16 = if idx <= 32 {
                (idx as i16) * 1008
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
        sai_tx: SaiTx,
        phase: u32,
    }

    #[shared]
    struct Shared {}

    // ── Init ─────────────────────────────────────────────────────────

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio2,
            pins,
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
            pins.p23,
            hal::sai::Pins {
                sync: pins.p27,
                bclk: pins.p26,
                data: pins.p7,
            },
            hal::sai::Pins {
                sync: pins.p20,
                bclk: pins.p21,
                data: pins.p8,
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

        // Pre-fill the FIFO with silence (zeros). Do NOT pre-fill with
        // tone data — TX is not enabled yet, so the FIFO just holds these
        // until the ISR starts producing real samples.
        for _ in 0..16 {
            sai_tx.write_frame(0, [0u32; 2]);
        }

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

        // ── Enable TX & SAI1 interrupt ──────────────────────────────
        // Enable FIFO Request Interrupt so the SAI1 ISR fires when
        // the FIFO needs more data.
        sai_tx.set_interrupts(hal::sai::Interrupts::FIFO_REQUEST);

        // Enable TX now — FIFO is still full (pre-filled above, not
        // draining because TX was deferred until here).
        sai_tx.set_enable(true);

        // Unmask SAI1 interrupt
        unsafe { cortex_m::peripheral::NVIC::unmask(ral::interrupt::SAI1) };

        (
            Shared {},
            Local {
                led,
                sai_tx,
                phase: 0,
            },
        )
    }

    // ── Idle: blink LED to prove firmware is alive ─────────────────

    #[idle(local = [led])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            cx.local.led.toggle();
            // ~500 ms at 600 MHz  (3 cycles per iteration)
            cortex_m::asm::delay(100_000_000);
        }
    }

    // ── SAI1 FIFO-request ISR ────────────────────────────────────────

    #[task(binds = SAI1, local = [sai_tx, phase], priority = 2)]
    fn sai1_isr(cx: sai1_isr::Context) {
        let sai_tx = cx.local.sai_tx;
        let phase = cx.local.phase;

        // Clear any error flags
        let status = sai_tx.status();
        if status.contains(hal::sai::Status::FIFO_ERROR) {
            sai_tx.clear_status(hal::sai::Status::FIFO_ERROR);
        }

        // Push frames into the FIFO.
        // The watermark is 16 words. When the IRQ fires (FIFO ≤ 16),
        // there are at least 16 free slots (FIFO depth is 32).
        // Each stereo frame = 2 words, so write 8 frames = 16 words
        // to fill the FIFO without overflow.  Writing more would
        // silently drop samples, causing phase jumps / chirps.
        for _ in 0..8 {
            let idx = ((*phase >> 8) & 0xFF) as usize;
            let sample = WAVE_TABLE[idx];
            // 16-bit sample → upper 16 bits of 32-bit I2S word (MSB-aligned).
            let sample32 = (sample as u16 as u32) << 16;
            sai_tx.write_frame(0, [sample32; 2]);
            *phase = phase.wrapping_add(PHASE_INC);
        }
    }

    // ── Tone generation constant ─────────────────────────────────────

    /// Phase-accumulator increment for 440 Hz @ Fs ≈ 44 117 Hz.
    /// 440 * 65536 / 44118 ≈ 653
    const PHASE_INC: u32 = 653;

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
