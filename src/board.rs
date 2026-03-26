//! Pre-configured board resources.
//!
//! The `board` module conveniently exposes drivers and hardware resources.
//! It configures peripheral clocks and power so that you can simply use the
//! board's peripherals.
//!
//! # Getting started
//!
//! Create board resources depending on your board:
//!
//! - Teensy 4.0 boards should use [`t40`].
//! - Teensy 4.1 boards should use [`t41`].
//! - Teensy MicroMod boards should use [`tmm`].
//!
//! The produced resources are nearly the same, except for the pins.
//!
//! ```no_run
//! use teensy4_bsp as bsp;
//! use bsp::board;
//!
//! let resources = board::t40(board::instances());
//! ```
//!
//! Resources take ownership of all peripheral instances `Instances`. Use [`instances()`] to
//! acquire these once.
//!
//! > Note that if you're using RTIC, you can let the RTIC framework allocate
//! > the instances. The examples included in this package demonstrate this pattern.
//!
//! # Resource categories
//!
//! The objects in [`Resources`] have two main categories:
//!
//! 1. ready-to-go drivers, like `gpt1` and `dma`, which are unlikely to change no matter your
//!    hardware setup.
//! 2. lower-level resources, like `pins` and `lpuart3`, which you can compose to create drivers
//!    specific for your hardware setup.
//!
//! Given the Teensy 4's I/O flexibility, it would be difficult to expose all features as ready-to-go
//! drivers. Instead, `board` tries to help you initialize drivers for your hardware
//! setup. For example,
//!
//! - use [`led`] to prepare the on-board LED.
//! - use [`lpspi`] to prepare SPI peripherals.
//! - use [`lpuart`] to prepare serial peripherals.
//! - use [`lpi2c`] to prepare I2C peripherals.
//!
//! For the complete list of helper functions, study the rest of this API documentation and their
//! examples. And for the complete driver documentation, see [the `hal` module](crate::hal) and
//! its submodules.
//!
//! These helper functions assume that you're following the `board`'s clock (and power) policy.
//!
//! # Clock policy
//!
//! Once [`t40`] and [`t41`] return, the clocks of all returned resources are running, and clock gates are
//! enabled. Essentially, `board` handles CCM configurations so that you can use peripherals with reasonable
//! settings.
//!
//! `board` exposes constants to help you understand those clock frequencies. You should
//! use these frequencies, along with your own settings, to describe certain types of objects, like
//! blocking delays.
//!
//! The example below shows how a user configures a GPT to meet a given GPT frequency.
//!
//! ```no_run
//! # use teensy4_bsp as bsp;
//! # use bsp::board;
//! use bsp::hal::{gpt, timer::Blocking};
//!
//! // Given this GPT clock source...
//! const GPT_CLOCK_SOURCE: gpt::ClockSource = gpt::ClockSource::PeripheralClock;
//! // ...and this GPT-specific divider...
//! const GPT_DIVIDER: u32 = 8;
//! /// ...the GPT frequency is
//! const GPT_FREQUENCY: u32 = board::PERCLK_FREQUENCY / GPT_DIVIDER;
//!
//! let board::Resources{ mut gpt1, .. }  = board::t40(board::instances());
//!
//! gpt1.set_clock_source(GPT_CLOCK_SOURCE);
//! gpt1.set_divider(GPT_DIVIDER);
//!
//! let mut delay = Blocking::<_, { GPT_FREQUENCY }>::from_gpt(gpt1);
//! delay.block_ms(500);
//! ```
//!
//! Clock frequency constants, and all APIs depending on those constants, assume that you do not change the clock policy.
//! But for those who want extra control, `Resources` exposes the clock management peripherals. If you take this
//! route, you're responsible for ensuring your peripheral's clock configurations, and you should ignore the constants
//! exposed by this module.
//!
//! ## Clock details
//!
//! `board` makes a few guarantees about the clock policy. This section describes those guarantees.
//!
//! It's considered an API-breaking change to vary the frequencies and derivations of these clocks:
//!
//! - The ARM core frequency is documented in [`ARM_FREQUENCY`].
//! - PERCLK, the root clock for GPT and PIT timers, is documented in [`PERCLK_FREQUENCY`].
//! - The crystal oscillator is documented in [`XTAL_OSCILLATOR_FREQUENCY`].
//!
//! `board` does not touch the following root clock components. You should configure these components yourself.
//!
//! - FlexIO1 and FlexIO2 dividers and multiplexers.
//!
//! Other clock frequencies are exempt from this policy; they may change value or derivation without
//! notice. Nevertheless, if these clock frequencies / derivations change, `board` still guarantees that
//! the functions to set peripheral baud and clock rates will work as expected.
//!
//! By the time your code has access to [`Resources`], the clock gates for _all_ peripherals provided by
//! `Resources` is set to "on." If you're not using specific resources, you may configure their clock
//! gates to "off." `board` code that borrows or converts peripherals may assume that clock gates are
//! set to "on."
//!
//! If you're not using the BSP to prepare board `Resources`, consider using [`prepare_clocks_and_power`]
//! to realize the BSP's clock policy.
//!
//! # Naming conventions
//!
//! This module identifies peripherals by their hardware names. It does not use identifiers
//! from the Teensy 4 SDK and documentation. For instance, to name the serial peripheral
//! that uses pins 14 and 15,
//!
//! - this module uses the name "LPUART**2**".
//! - the Teensy 4 SDK / documentation uses the name "UART**3**".
//!
//! There's a few reasons for these differences:
//!
//! - It would be unintuitive to register a LPUART**2** *interrupt* for the peripheral named
//!   UART**3**. Since the BSP typically does not manage interrupt registration, this naming
//!   decision makes it easier for you to register and implement interrupts.
//! - It makes clear the peripheral implementation. You know that the serial peripheral
//!   supports low-power (LP) operation, and that it's not a FlexIO peripheral emulating a UART
//!   device.

use crate::{hal, pins, ral};
use core::sync::atomic::{AtomicBool, Ordering};
pub use hal::lpspi::Pins as LpspiPins;

/// Use [`instances()`] to safely acquire.
pub use ral::Instances;

/// Acquire peripheral instances.
///
/// These are the resources supplied to [a resource constructor](crate::board#getting-started).
/// They can also be used to initialize your own drivers.
///
/// ```
/// use teensy4_bsp as bsp;
/// use bsp::board;
///
/// let instances = board::instances();
/// ```
///
/// # Panics
///
/// Panics if the instances have already been taken from this function.
/// If you're using RTIC, know that RTIC may take these instances before
/// calling your `init` function.
///
/// ```should_panic
/// # use teensy4_bsp as bsp;
/// # use bsp::board;
/// let instances = board::instances();
/// // Sometime later...
/// board::instances(); // Panics! Instances already taken.
/// ```
///
/// # Safety
///
/// This function provides a safe, convenient way to acquire one [`Instances`] object.
/// It's considered safe for the use-cases that `teensy4-bsp` intends to support;
/// specifically, you're building one binary that represents the entire firmware
/// image, and that there's no other software which is acquiring these resources.
pub fn instances() -> Instances {
    static TAKEN: AtomicBool = AtomicBool::new(false);
    assert!(!TAKEN.swap(true, Ordering::SeqCst));
    unsafe { Instances::instances() }
}

pub use crate::clock_power::*;

/// Resources for a Teensy 4.0.
///
/// Use [`t40`] to construct this. The pins are specific to the Teensy 4.0.
pub type T40Resources = Resources<pins::t40::Pins>;
/// Resources for a Teensy 4.1.
///
/// Use [`t41`] to construct this. The pins are specific to the Teensy 4.1.
pub type T41Resources = Resources<pins::t41::Pins>;
/// Resources for a Teensy MicroMod.
///
/// Use [`tmm`] to construct this. The pins are specific to the Teensy MicroMod.
pub type TMMResources = Resources<pins::tmm::Pins>;

/// Resources constructed by the board.
///
/// The concrete `Pins` type depends on how this is constructed.
/// See the various `*Resources` aliases for more information.
#[non_exhaustive]
pub struct Resources<Pins> {
    /// Periodic interrupt timer channels.
    pub pit: hal::pit::Channels,
    /// General purpose timer 1.
    pub gpt1: hal::gpt::Gpt1,
    /// General purpose timer 2.
    pub gpt2: hal::gpt::Gpt2,
    /// GPIO1 port.
    pub gpio1: hal::gpio::Port<1>,
    /// GPIO2 port.
    pub gpio2: hal::gpio::Port<2>,
    /// GPIO3 port.
    pub gpio3: hal::gpio::Port<3>,
    /// GPIO4 port.
    pub gpio4: hal::gpio::Port<4>,
    /// USB1 instances.
    ///
    /// Use this to construct higher-level USB drivers, or to initialize the USB logger.
    pub usb: hal::usbd::Instances<1>,
    /// DMA channels.
    pub dma: [Option<hal::dma::channel::Channel>; hal::dma::CHANNEL_COUNT],
    /// The secure real-time counter.
    ///
    /// It's initially disabled, and you may enable it in your firmware.
    pub srtc: hal::snvs::srtc::Disabled,
    /// Core registers for the SNVS low-power domain.
    ///
    /// Use this with the SRTC and other SNVS LP components.
    pub snvs_lp_core: hal::snvs::LpCore,
    /// Clock control module.
    pub ccm: ral::ccm::CCM,
    /// Analog clock control module.
    pub ccm_analog: ral::ccm_analog::CCM_ANALOG,
    /// DCDC converter
    pub dcdc: ral::dcdc::DCDC,
    /// All available pins.
    pub pins: Pins,
    /// The register block for [`Lpi2c1`].
    pub lpi2c1: ral::lpi2c::LPI2C1,
    /// The register block for [`Lpi2c3`].
    pub lpi2c3: ral::lpi2c::LPI2C3,
    /// The register blocks for [`Lpspi1`].
    pub lpspi1: ral::lpspi::LPSPI1,
    /// The register blocks for [`Lpspi2`].
    pub lpspi2: ral::lpspi::LPSPI2,
    /// The register blocks for [`Lpspi3`].
    pub lpspi3: ral::lpspi::LPSPI3,
    /// The register block for [`Lpspi4`].
    pub lpspi4: ral::lpspi::LPSPI4,
    /// The register block for [`Lpuart6`].
    pub lpuart6: ral::lpuart::LPUART6,
    /// The register block for [`Lpuart4`].
    pub lpuart4: ral::lpuart::LPUART4,
    /// The register block for [`Lpuart2`].
    pub lpuart2: ral::lpuart::LPUART2,
    /// The register block for [`Lpuart3`].
    pub lpuart3: ral::lpuart::LPUART3,
    /// The register block for [`Lpuart8`].
    pub lpuart8: ral::lpuart::LPUART8,
    /// The register block for [`Lpuart1`].
    pub lpuart1: ral::lpuart::LPUART1,
    /// The register block for [`Lpuart5`].
    pub lpuart5: ral::lpuart::LPUART5,
    /// The register block for [`Lpuart7`].
    pub lpuart7: ral::lpuart::LPUART7,
    /// FlexPWM1 components.
    pub flexpwm1: (hal::flexpwm::Pwm<1>, hal::flexpwm::Submodules<1>),
    /// FlexPWM2 components.
    pub flexpwm2: (hal::flexpwm::Pwm<2>, hal::flexpwm::Submodules<2>),
    /// FlexPWM3 components.
    pub flexpwm3: (hal::flexpwm::Pwm<3>, hal::flexpwm::Submodules<3>),
    /// FlexPWM4 components.
    pub flexpwm4: (hal::flexpwm::Pwm<4>, hal::flexpwm::Submodules<4>),
    /// The FlexIO1 register block.
    pub flexio1: ral::flexio::FLEXIO1,
    /// The FlexIO2 register block.
    pub flexio2: ral::flexio::FLEXIO2,
    /// The FlexIO3 register block.
    pub flexio3: ral::flexio::FLEXIO3,
    /// The register block for ADC1.
    ///
    /// ADC drivers constructed by `board` use a pre-configured clock and divisor. To change
    /// this configuration, call `release()` to acquire the register block, then re-construct
    /// the driver.
    pub adc1: hal::adc::Adc<1>,
    /// The register block for ADC2.
    pub adc2: hal::adc::Adc<2>,
    /// True random number generator.
    pub trng: hal::trng::Trng,
    /// Temperature monitor of the core.
    pub tempmon: hal::tempmon::TempMon,
    /// The register block for SAI1 (I2S audio).
    ///
    /// SAI1 is the primary audio interface used by the Teensy Audio Shield.
    pub sai1: ral::sai::SAI1,
    /// The register block for SAI2.
    pub sai2: ral::sai::SAI2,
    /// The register block for SAI3.
    pub sai3: ral::sai::SAI3,
    /// The IOMUXC general purpose register block.
    pub iomuxc_gpr: ral::iomuxc_gpr::IOMUXC_GPR,
    /// The FlexSPI2 peripheral instance.
    ///
    /// On the Teensy 4.1, FlexSPI2 is connected to the on-board PSRAM
    /// chip(s). Pass this to [`initialize_psram`] (or [`init_psram`])
    /// to initialize PSRAM; doing so consumes the instance, preventing
    /// accidental misuse of the peripheral after initialization.
    pub flexspi2: ral::flexspi::FLEXSPI2,
}

/// The board's dedicated LED.
pub type Led = hal::gpio::Output<pins::common::P13>;

/// Create the board's LED.
///
/// ```no_run
/// use teensy4_bsp as bsp;
/// use bsp::board;
///
/// let board::Resources { mut gpio2, pins, .. }
///     = board::t40(board::instances());
///
/// let led = board::led(&mut gpio2, pins.p13);
/// ```
pub fn led(gpio2: &mut hal::gpio::Port<2>, p13: pins::common::P13) -> Led {
    gpio2.output(p13)
}

/// Create a LPI2C peripheral.
///
/// Consider using explicit type annotations, as demonstrated below,
/// to simplify any errors from the compiler.
///
/// This helper assumes that the actual LPI2C clock frequency equals [`LPI2C_FREQUENCY`].
///
/// ```no_run
/// use teensy4_bsp as bsp;
/// use bsp::board;
///
/// let board::Resources { lpi2c3, pins, ..}
///     = board::t40(board::instances());
///
/// let mut lpi2c: board::Lpi2c3 = board::lpi2c(
///     lpi2c3,
///     pins.p16,
///     pins.p17,
///     board::Lpi2cClockSpeed::KHz400,
/// );
/// ```
pub fn lpi2c<Scl, Sda, const N: u8>(
    instance: ral::lpi2c::Instance<N>,
    scl: Scl,
    sda: Sda,
    clock_speed: Lpi2cClockSpeed,
) -> hal::lpi2c::Lpi2c<hal::lpi2c::Pins<Scl, Sda>, N>
where
    Scl: hal::iomuxc::lpi2c::Pin<
        Signal = hal::iomuxc::lpi2c::Scl,
        Module = hal::iomuxc::consts::Const<N>,
    >,
    Sda: hal::iomuxc::lpi2c::Pin<
        Signal = hal::iomuxc::lpi2c::Sda,
        Module = hal::iomuxc::consts::Const<N>,
    >,
{
    hal::lpi2c::Lpi2c::new(
        instance,
        hal::lpi2c::Pins { scl, sda },
        &lpi2c_baud(clock_speed),
    )
}

/// LPI2C1 peripheral.
///
/// - Pin 19 is the clock line.
/// - Pin 18 is the data line.
///
/// Use [`lpi2c`] to create this driver.
pub type Lpi2c1 = hal::lpi2c::Lpi2c<hal::lpi2c::Pins<pins::common::P19, pins::common::P18>, 1>;

/// LPI2C3 peripheral.
///
/// - Pin 16 is the clock line.
/// - Pin 17 is the data line.
///
/// Use [`lpi2c`] to create this driver.
pub type Lpi2c3 = hal::lpi2c::Lpi2c<hal::lpi2c::Pins<pins::common::P16, pins::common::P17>, 3>;

/// LPSPI1 peripheral.
///
/// - SDO:  GPIO_SD_B0_02 (p43) or GPIO_EMC_28 (p50)
/// - SDI:  GPIO_SD_B0_03 (p42) or GPIO_EMC_29 (p54)
/// - SCK:  GPIO_SD_B0_00 (p45) or GPIO_EMC_27 (p49)
///
/// Use [`lpspi`] to create this driver.
pub type Lpspi1<SDO, SDI, SCK> = hal::lpspi::Lpspi<LpspiPins<SDO, SDI, SCK>, 1>;

/// LPSPI2 peripheral.
///
/// - SDO:  GPIO_SD_B1_08 or GPIO_EMC_02
/// - SDI:  GPIO_SD_B1_09 or GPIO_EMC_03
/// - SCK:  GPIO_SD_B1_07 or GPIO_EMC_00
///
/// Use [`lpspi`] to create this driver.
pub type Lpspi2<SDO, SDI, SCK> = hal::lpspi::Lpspi<LpspiPins<SDO, SDI, SCK>, 2>;

/// LPSPI3 peripheral.
///
/// CS and SDI have two options each for which pin to use.
///
/// - Pin 26 is data out (SDO).
/// - Pin 39 or 1 is data in (SDI).
/// - Pin 27 is clock (SCK).
///
/// Use [`lpspi`] to create this driver.
pub type Lpspi3<SDI> = hal::lpspi::Lpspi<LpspiPins<pins::common::P26, SDI, pins::common::P27>, 3>;

/// LPSPI4 peripheral.
///
/// - Pin 11 is data out (SDO).
/// - Pin 12 is data in (SDI).
/// - Pin 13 is clock (SCK).
///
/// Use [`lpspi`] to create this driver.
pub type Lpspi4 =
    hal::lpspi::Lpspi<LpspiPins<pins::common::P11, pins::common::P12, pins::common::P13>, 4>;

/// Create a LPSPI peripheral.
///
/// `baud` is the SCK frequency, in Hz. Consider using explicit type annotations, as demonstrated
/// below, to simplify any errors from the compiler.
///
/// This helper assumes that the LPSPI clock frequency equals [`LPSPI_FREQUENCY`].
///
/// ```no_run
/// use teensy4_bsp as bsp;
/// use bsp::board;
///
/// let board::T40Resources { lpspi4, pins, .. }
///     = board::t40(board::instances());
///
/// let mut lpspi4: board::Lpspi4 = board::lpspi(
///     lpspi4,
///     board::LpspiPins {
///         sdo: pins.p11,
///         sdi: pins.p12,
///         sck: pins.p13,
///     },
///     1_000_000,
/// );
/// ```
pub fn lpspi<Sdo, Sdi, Sck, const N: u8>(
    instance: ral::lpspi::Instance<N>,
    pins: LpspiPins<Sdo, Sdi, Sck>,
    baud: u32,
) -> hal::lpspi::Lpspi<LpspiPins<Sdo, Sdi, Sck>, N>
where
    Sdo: hal::iomuxc::lpspi::Pin<
        Signal = hal::iomuxc::lpspi::Sdo,
        Module = hal::iomuxc::consts::Const<N>,
    >,
    Sdi: hal::iomuxc::lpspi::Pin<
        Signal = hal::iomuxc::lpspi::Sdi,
        Module = hal::iomuxc::consts::Const<N>,
    >,
    Sck: hal::iomuxc::lpspi::Pin<
        Signal = hal::iomuxc::lpspi::Sck,
        Module = hal::iomuxc::consts::Const<N>,
    >,
{
    let mut spi = hal::lpspi::Lpspi::new(instance, pins);
    spi.disabled(|spi| spi.set_clock_hz(LPSPI_FREQUENCY, baud));
    spi
}

/// Create a LPUART peripheral.
///
/// The specific peripheral you're creating depends on
///
/// - the type of `instance`.
/// - the TX and RX pins.
/// - the return type, which can be explicitly annotated.
///
/// Consider using an explicit type annotation to help catch any
/// programming errors.
///
/// This helper assumes that the UART clock frequency equals [`UART_FREQUENCY`].
///
/// ```no_run
/// use teensy4_bsp as bsp;
/// use bsp::board;
///
/// let board::T40Resources { lpuart6, lpuart2, pins, .. }
///     = board::t40(board::instances());
///
/// // Explicit type:
/// let mut lpuart6: board::Lpuart6 = board::lpuart(
///     lpuart6,
///     pins.p1,
///     pins.p0,
///     115200,
/// );
///
/// // Implicit type:
/// let mut lpuart2 = board::lpuart(lpuart2, pins.p14, pins.p15, 9600);
/// ```
pub fn lpuart<Tx, Rx, const N: u8>(
    instance: ral::lpuart::Instance<N>,
    tx: Tx,
    rx: Rx,
    baud: u32,
) -> hal::lpuart::Lpuart<hal::lpuart::Pins<Tx, Rx>, N>
where
    Tx: hal::iomuxc::lpuart::Pin<
        Direction = hal::iomuxc::lpuart::Tx,
        Module = hal::iomuxc::consts::Const<N>,
    >,
    Rx: hal::iomuxc::lpuart::Pin<
        Direction = hal::iomuxc::lpuart::Rx,
        Module = hal::iomuxc::consts::Const<N>,
    >,
{
    let mut uart = hal::lpuart::Lpuart::new(instance, hal::lpuart::Pins { tx, rx });
    uart.disable(|uart| uart.set_baud(&lpuart_baud(baud)));
    uart
}

/// LPUART6 peripheral.
///
/// - Pin 1 is TX.
/// - Pin 0 is RX.
///
/// Use [`lpuart`] to create this driver.
pub type Lpuart6 = hal::lpuart::Lpuart<hal::lpuart::Pins<pins::common::P1, pins::common::P0>, 6>;

/// LPUART4 peripheral.
///
/// - Pin 8 is TX.
/// - Pin 7 is RX.
///
/// Use [`lpuart`] to create this driver.
pub type Lpuart4 = hal::lpuart::Lpuart<hal::lpuart::Pins<pins::common::P8, pins::common::P7>, 4>;

/// LPUART2 peripheral.
///
/// - Pin 14 is TX.
/// - Pin 15 is RX.
///
/// Use [`lpuart`] to create this driver.
pub type Lpuart2 = hal::lpuart::Lpuart<hal::lpuart::Pins<pins::common::P14, pins::common::P15>, 2>;

/// LPUART3 peripheral.
///
/// - Pin 17 is TX.
/// - Pin 16 is RX.
///
/// Use [`lpuart`] to create this driver.
pub type Lpuart3 = hal::lpuart::Lpuart<hal::lpuart::Pins<pins::common::P17, pins::common::P16>, 3>;

/// LPUART8 peripheral.
///
/// - Pin 20 is TX.
/// - Pin 21 is RX.
///
/// Use [`lpuart`] to create this driver.
pub type Lpuart8 = hal::lpuart::Lpuart<hal::lpuart::Pins<pins::common::P20, pins::common::P21>, 8>;

/// LPUART1 peripheral.
///
/// - Pin 24 is TX.
/// - Pin 25 is RX.
///
/// Use [`lpuart`] to create this driver.
pub type Lpuart1 = hal::lpuart::Lpuart<hal::lpuart::Pins<pins::common::P24, pins::common::P25>, 1>;

/// LPUART7 peripheral.
///
/// - Pin 29 is TX.
/// - Pin 28 is RX.
///
/// Use [`lpuart`] to create this driver.
pub type Lpuart7 = hal::lpuart::Lpuart<hal::lpuart::Pins<pins::common::P29, pins::common::P28>, 7>;

/// LPUART5 peripheral, available on the Teensy 4.1.
///
/// - Pin 35 is TX.
/// - Pin 34 is RX.
///
/// Use [`lpuart`] to create this driver.
pub type Lpuart5 = hal::lpuart::Lpuart<hal::lpuart::Pins<pins::t41::P35, pins::t41::P34>, 5>;

fn prepare_resources<Pins>(
    mut instances: Instances,
    from_pads: impl FnOnce(hal::iomuxc::pads::Pads) -> Pins,
) -> Resources<Pins> {
    prepare_clocks_and_power(
        &mut instances.CCM,
        &mut instances.CCM_ANALOG,
        &mut instances.DCDC,
    );
    let iomuxc = hal::iomuxc::into_pads(instances.IOMUXC);
    let pins = from_pads(iomuxc);

    // Stop timers in debug mode.
    ral::modify_reg!(ral::pit, instances.PIT, MCR, FRZ: FRZ_1);
    let pit = hal::pit::new(instances.PIT);

    let mut gpt1 = hal::gpt::Gpt::new(instances.GPT1);
    gpt1.disable();

    let mut gpt2 = hal::gpt::Gpt::new(instances.GPT2);
    gpt2.disable();

    let hal::snvs::Snvs {
        low_power:
            hal::snvs::LowPower {
                core: snvs_lp_core,
                srtc,
                ..
            },
        ..
    } = hal::snvs::new(instances.SNVS);

    let adc1 = hal::adc::Adc::new(
        instances.ADC1,
        hal::adc::ClockSelect::ADACK,
        hal::adc::ClockDivision::Div8,
    );
    let adc2 = hal::adc::Adc::new(
        instances.ADC2,
        hal::adc::ClockSelect::ADACK,
        hal::adc::ClockDivision::Div8,
    );

    let trng = hal::trng::Trng::new(instances.TRNG, Default::default(), Default::default());
    let tempmon = hal::tempmon::TempMon::with_measure_freq(instances.TEMPMON, 0x1000);

    Resources {
        pit,
        gpt1,
        gpt2,
        gpio1: hal::gpio::Port::new(instances.GPIO1),
        gpio2: hal::gpio::Port::new(instances.GPIO2),
        gpio3: hal::gpio::Port::new(instances.GPIO3),
        gpio4: hal::gpio::Port::new(instances.GPIO4),
        usb: hal::usbd::Instances {
            usb: instances.USB1,
            usbphy: instances.USBPHY1,
            usbnc: instances.USBNC1,
        },
        dma: hal::dma::channels(instances.DMA, instances.DMAMUX),
        srtc,
        snvs_lp_core,
        ccm: instances.CCM,
        ccm_analog: instances.CCM_ANALOG,
        dcdc: instances.DCDC,
        pins,
        lpi2c1: instances.LPI2C1,
        lpi2c3: instances.LPI2C3,
        lpspi1: instances.LPSPI1,
        lpspi2: instances.LPSPI2,
        lpspi3: instances.LPSPI3,
        lpspi4: instances.LPSPI4,
        lpuart6: instances.LPUART6,
        lpuart4: instances.LPUART4,
        lpuart2: instances.LPUART2,
        lpuart3: instances.LPUART3,
        lpuart8: instances.LPUART8,
        lpuart1: instances.LPUART1,
        lpuart7: instances.LPUART7,
        lpuart5: instances.LPUART5,
        flexio1: instances.FLEXIO1,
        flexio2: instances.FLEXIO2,
        flexio3: instances.FLEXIO3,
        flexpwm1: hal::flexpwm::new(instances.PWM1),
        flexpwm2: hal::flexpwm::new(instances.PWM2),
        flexpwm3: hal::flexpwm::new(instances.PWM3),
        flexpwm4: hal::flexpwm::new(instances.PWM4),
        adc1,
        adc2,
        trng,
        tempmon,
        sai1: instances.SAI1,
        sai2: instances.SAI2,
        sai3: instances.SAI3,
        iomuxc_gpr: instances.IOMUXC_GPR,
        flexspi2: instances.FLEXSPI2,
    }
}

/// Create resources for the Teensy 4.0 board.
///
/// Note that the peripheral instances acquired by RTIC -- named `device` in the
/// `init::Context` object -- can be used as the argument to this function.
pub fn t40(instances: impl Into<Instances>) -> T40Resources {
    prepare_resources(instances.into(), pins::t40::from_pads)
}

/// Create resources for the Teensy 4.1 board.
///
/// Note that the peripheral instances acquired by RTIC -- named `device` in the
/// `init::Context` object -- can be used as the argument to this function.
pub fn t41(instances: impl Into<Instances>) -> T41Resources {
    prepare_resources(instances.into(), pins::t41::from_pads)
}

/// Create resources for the Teensy MicroMod board.
///
/// Note that the peripheral instances acquired by RTIC -- named `device` in the
/// `init::Context` object -- can be used as the argument to this function.
pub fn tmm(instances: impl Into<Instances>) -> TMMResources {
    prepare_resources(instances.into(), pins::tmm::from_pads)
}

// --------------------------------------------------------------------------
//  PSRAM (FlexSPI2) board support — Teensy 4.1
// --------------------------------------------------------------------------

/// Zero-sized token proving that PSRAM has been initialized.
///
/// Returned by [`initialize_psram`] on success. Pass this to
/// [`PsramStatic::take`](crate::PsramStatic::take) to obtain
/// references to PSRAM-backed statics declared with
/// [`psram_static!`](crate::psram_static).
///
/// `Copy` so it can be passed to `.take()` on multiple independent
/// statics. One-shot safety is enforced per-static via `AtomicBool`,
/// not by the token.
#[derive(Clone, Copy, Debug)]
pub struct PsramToken(());

/// Errors from [`initialize_psram`].
#[derive(Debug)]
#[non_exhaustive]
pub enum PsramError {
    /// No PSRAM chip was detected on FlexSPI2.
    NotDetected,
    /// The `.psram.data` section is larger than the detected PSRAM capacity.
    SectionOverflow {
        /// Size of the `.psram.data` section in bytes.
        section_size: usize,
        /// Detected PSRAM capacity in bytes.
        detected_size: usize,
    },
}

/// Initialize PSRAM and copy initializer data from flash.
///
/// Configures FlexSPI2, detects installed PSRAM, and copies the
/// `.psram.data` section contents from flash (LMA) into PSRAM (VMA).
/// On success, returns a [`PsramToken`] that can be used to access
/// PSRAM-backed statics.
///
/// Consumes the `FLEXSPI2` instance to prevent further direct use
/// of the peripheral after initialization.
///
/// # Errors
///
/// Returns [`PsramError::NotDetected`] if no PSRAM chip responds.
/// Returns [`PsramError::SectionOverflow`] if the `.psram.data`
/// section is larger than the detected capacity.
///
/// # Safety
///
/// Must be called once during early initialization, before any PSRAM
/// access. Modifies IOMUXC and FlexSPI2 peripheral registers. The
/// FlexSPI2 clock must already be configured (this is done
/// automatically by [`prepare_clocks_and_power`]).
#[cfg(feature = "psram")]
pub unsafe fn initialize_psram(
    flexspi2: ral::flexspi::FLEXSPI2,
) -> Result<PsramToken, PsramError> {
    let size_mb = init_psram(flexspi2);
    if size_mb == 0 {
        return Err(PsramError::NotDetected);
    }

    let detected_bytes = size_mb as usize * 1024 * 1024;

    // Linker-provided symbols from psram.x.
    //
    // Assumptions (see docs/psram-api-design.md "Assumptions and Fragility"):
    // - psram.x defines these symbols via the .psram.data section.
    // - psram.x uses `AT> FLASH` which requires t4link.x (imxrt-rt) to
    //   define a FLASH region. If imxrt-rt renames it, linking fails.
    // - psram.x uses `INSERT AFTER .uninit` which requires t4link.x to
    //   define a .uninit section. If imxrt-rt removes it, linking fails.
    // - VMA is at 0x7000_0000 (PSRAM memory-mapped base).
    extern "C" {
        static __spsram_data: u8;
        static __epsram_data: u8;
        static __sipsram_data: u8;
    }

    let vma_start = &__spsram_data as *const u8;
    let vma_end = &__epsram_data as *const u8;
    let lma_start = &__sipsram_data as *const u8;
    let section_size = vma_end as usize - vma_start as usize;

    if section_size > detected_bytes {
        return Err(PsramError::SectionOverflow {
            section_size,
            detected_size: detected_bytes,
        });
    }

    // Copy initializer data from flash to PSRAM.
    if section_size > 0 {
        core::ptr::copy_nonoverlapping(lma_start, vma_start as *mut u8, section_size);
    }

    Ok(PsramToken(()))
}

/// PSRAM memory-mapped base address.
///
/// After [`init_psram`] returns successfully, PSRAM is accessible
/// as normal memory starting at this address.
pub const PSRAM_BASE: usize = 0x7000_0000;

/// Maximum PSRAM size in bytes (8 MB per chip, up to 16 MB with two chips).
pub const PSRAM_MAX_SIZE: usize = 16 * 1024 * 1024;

// --- FlexSPI LUT instruction encoding ---
// These constants encode FlexSPI protocol instructions, not register fields.

const FLEXSPI_CMD_SDR: u32 = 0x01;
const FLEXSPI_RADDR_SDR: u32 = 0x02;
const FLEXSPI_READ_SDR: u32 = 0x09;
const FLEXSPI_WRITE_SDR: u32 = 0x08;
const FLEXSPI_DUMMY_SDR: u32 = 0x0C;

const FLEXSPI_PADS_1: u32 = 0;
const FLEXSPI_PADS_4: u32 = 2;

const fn lut_instr(opcode: u32, pads: u32, operand: u32) -> u32 {
    (opcode << 10) | (pads << 8) | operand
}

const fn lut_word(instr0: u32, instr1: u32) -> u32 {
    instr0 | (instr1 << 16)
}

unsafe fn flexspi2_command(
    flexspi2: &ral::flexspi::Instance<2>,
    seq_index: u32,
    addr: u32,
) {
    ral::write_reg!(ral::flexspi, flexspi2, IPCR0, SFAR: addr);
    ral::write_reg!(ral::flexspi, flexspi2, IPCR1, ISEQID: seq_index);
    ral::write_reg!(ral::flexspi, flexspi2, IPCMD, TRG: 1);
    while ral::read_reg!(ral::flexspi, flexspi2, INTR, IPCMDDONE) == 0 {}
    ral::write_reg!(ral::flexspi, flexspi2, INTR, IPCMDDONE: 1);
}

unsafe fn flexspi2_psram_id(
    flexspi2: &ral::flexspi::Instance<2>,
    addr: u32,
) -> u32 {
    ral::write_reg!(ral::flexspi, flexspi2, IPCR0, SFAR: addr);
    ral::write_reg!(ral::flexspi, flexspi2, IPCR1, ISEQID: 3, IDATSZ: 4);
    ral::write_reg!(ral::flexspi, flexspi2, IPCMD, TRG: 1);
    while ral::read_reg!(ral::flexspi, flexspi2, INTR, IPCMDDONE) == 0 {}
    let id = flexspi2.RFDR[0].read();
    ral::write_reg!(ral::flexspi, flexspi2, INTR, IPCMDDONE: 1, IPRXWA: 1);
    id
}

unsafe fn flexspi2_psram_size(
    flexspi2: &ral::flexspi::Instance<2>,
    addr: u32,
) -> u8 {
    flexspi2_command(flexspi2, 0, addr);
    flexspi2_command(flexspi2, 1, addr);
    flexspi2_command(flexspi2, 2, addr);

    let id = flexspi2_psram_id(flexspi2, addr);

    // Parse PSRAM device ID (protocol-level, not register fields)
    match id & 0xFFFF {
        0x5D0D => 8,    // AP/Lyontek
        0x5D9D => {     // ISSI
            match (id >> 21) & 0x7 {
                0b011 => 8,
                0b100 => 16,
                _ => 0,
            }
        }
        _ => 0,
    }
}

/// Initialize PSRAM via FlexSPI2 on the Teensy 4.1.
///
/// Configures the FlexSPI2 peripheral, pad muxing, and clock for the
/// on-board PSRAM chip (APS6404L-3SQR or compatible). After a
/// successful return, PSRAM is memory-mapped at [`PSRAM_BASE`]
/// (`0x7000_0000`).
///
/// Consumes the `flexspi2` instance to prevent further direct access
/// to the peripheral after PSRAM is initialized.
///
/// Returns the total PSRAM size in megabytes (typically 8 or 16),
/// or 0 if no PSRAM chip was detected.
///
/// # Safety
///
/// Must be called once during early initialization, before any PSRAM
/// access. Modifies IOMUXC and FlexSPI2 peripheral registers. The
/// FlexSPI2 clock must already be configured (this is done
/// automatically by [`prepare_clocks_and_power`]).
pub unsafe fn init_psram(flexspi2: ral::flexspi::FLEXSPI2) -> u8 {
    let iomuxc = ral::iomuxc::IOMUXC::instance();

    // ---- Pad configuration ----
    // 8 pads on GPIO_EMC_22..29, all ALT8 for FlexSPI2 port A.
    // All high-speed: HYS=1, DSE=7(R0/7), SPEED=3(200MHz), SRE=1(fast).
    // Pull-up varies per pin function.

    // EMC_22 (FlexSPI2_A_SS1_B): 100K pull-up
    ral::write_reg!(ral::iomuxc, iomuxc, SW_PAD_CTL_PAD_GPIO_EMC_22,
        HYS: 1, PUS: 2, PUE: 1, PKE: 1, DSE: 7, SPEED: 3, SRE: 1);
    // EMC_23 (FlexSPI2_A_DQS): keeper only
    ral::write_reg!(ral::iomuxc, iomuxc, SW_PAD_CTL_PAD_GPIO_EMC_23,
        HYS: 1, PKE: 1, DSE: 7, SPEED: 3, SRE: 1);
    // EMC_24 (FlexSPI2_A_SS0_B): 100K pull-up
    ral::write_reg!(ral::iomuxc, iomuxc, SW_PAD_CTL_PAD_GPIO_EMC_24,
        HYS: 1, PUS: 2, PUE: 1, PKE: 1, DSE: 7, SPEED: 3, SRE: 1);
    // EMC_25 (FlexSPI2_A_SCLK): no pull
    ral::write_reg!(ral::iomuxc, iomuxc, SW_PAD_CTL_PAD_GPIO_EMC_25,
        HYS: 1, DSE: 7, SPEED: 3, SRE: 1);
    // EMC_26..29 (FlexSPI2_A_DATA0..3): 47K pull-up
    ral::write_reg!(ral::iomuxc, iomuxc, SW_PAD_CTL_PAD_GPIO_EMC_26,
        HYS: 1, PUS: 1, PUE: 1, PKE: 1, DSE: 7, SPEED: 3, SRE: 1);
    ral::write_reg!(ral::iomuxc, iomuxc, SW_PAD_CTL_PAD_GPIO_EMC_27,
        HYS: 1, PUS: 1, PUE: 1, PKE: 1, DSE: 7, SPEED: 3, SRE: 1);
    ral::write_reg!(ral::iomuxc, iomuxc, SW_PAD_CTL_PAD_GPIO_EMC_28,
        HYS: 1, PUS: 1, PUE: 1, PKE: 1, DSE: 7, SPEED: 3, SRE: 1);
    ral::write_reg!(ral::iomuxc, iomuxc, SW_PAD_CTL_PAD_GPIO_EMC_29,
        HYS: 1, PUS: 1, PUE: 1, PKE: 1, DSE: 7, SPEED: 3, SRE: 1);

    // Mux: ALT8 + SION for all FlexSPI2 pads
    ral::write_reg!(ral::iomuxc, iomuxc, SW_MUX_CTL_PAD_GPIO_EMC_22, MUX_MODE: 8, SION: 1);
    ral::write_reg!(ral::iomuxc, iomuxc, SW_MUX_CTL_PAD_GPIO_EMC_23, MUX_MODE: 8, SION: 1);
    ral::write_reg!(ral::iomuxc, iomuxc, SW_MUX_CTL_PAD_GPIO_EMC_24, MUX_MODE: 8, SION: 1);
    ral::write_reg!(ral::iomuxc, iomuxc, SW_MUX_CTL_PAD_GPIO_EMC_25, MUX_MODE: 8, SION: 1);
    ral::write_reg!(ral::iomuxc, iomuxc, SW_MUX_CTL_PAD_GPIO_EMC_26, MUX_MODE: 8, SION: 1);
    ral::write_reg!(ral::iomuxc, iomuxc, SW_MUX_CTL_PAD_GPIO_EMC_27, MUX_MODE: 8, SION: 1);
    ral::write_reg!(ral::iomuxc, iomuxc, SW_MUX_CTL_PAD_GPIO_EMC_28, MUX_MODE: 8, SION: 1);
    ral::write_reg!(ral::iomuxc, iomuxc, SW_MUX_CTL_PAD_GPIO_EMC_29, MUX_MODE: 8, SION: 1);

    // Input daisy chains: select GPIO_EMC path
    ral::write_reg!(ral::iomuxc, iomuxc, FLEXSPI2_IPP_IND_DQS_FA_SELECT_INPUT, DAISY: 1);
    ral::write_reg!(ral::iomuxc, iomuxc, FLEXSPI2_IPP_IND_IO_FA_BIT0_SELECT_INPUT, DAISY: 1);
    ral::write_reg!(ral::iomuxc, iomuxc, FLEXSPI2_IPP_IND_IO_FA_BIT1_SELECT_INPUT, DAISY: 1);
    ral::write_reg!(ral::iomuxc, iomuxc, FLEXSPI2_IPP_IND_IO_FA_BIT2_SELECT_INPUT, DAISY: 1);
    ral::write_reg!(ral::iomuxc, iomuxc, FLEXSPI2_IPP_IND_IO_FA_BIT3_SELECT_INPUT, DAISY: 1);
    ral::write_reg!(ral::iomuxc, iomuxc, FLEXSPI2_IPP_IND_SCK_FA_SELECT_INPUT, DAISY: 1);

    // ---- FlexSPI2 peripheral configuration ----

    // Disable module during configuration
    ral::modify_reg!(ral::flexspi, flexspi2, MCR0, MDIS: 1);

    // MCR0: max timeouts, RXCLKSRC=1 (loopback from DQS pad)
    ral::modify_reg!(ral::flexspi, flexspi2, MCR0,
        AHBGRANTWAIT: 0xFF, IPGRANTWAIT: 0xFF,
        SCKFREERUNEN: 0, COMBINATIONEN: 0, DOZEEN: 0, HSEN: 0,
        ATDFEN: 0, ARDFEN: 0, RXCLKSRC: 1, MDIS: 1, SWRESET: 0);

    ral::write_reg!(ral::flexspi, flexspi2, MCR1,
        SEQWAIT: 0xFFFF, AHBBUSWAIT: 0xFFFF);

    ral::modify_reg!(ral::flexspi, flexspi2, MCR2,
        RESUMEWAIT: 0x20, SCKBDIFFOPT: 0, SAMEDEVICEEN: 0,
        CLRLEARNPHASE: 0, CLRAHBBUFOPT: 0);

    ral::modify_reg!(ral::flexspi, flexspi2, AHBCR,
        READADDROPT: 0, PREFETCHEN: 0, BUFFERABLEEN: 0, CACHABLEEN: 0);

    // AHB RX buffers 0/1: prefetch enabled, 64 x 64-bit = 512 bytes each
    ral::modify_reg!(ral::flexspi, flexspi2, AHBRXBUF0CR0,
        PREFETCHEN: 1, PRIORITY: 0, MSTRID: 0, BUFSZ: 64);
    ral::modify_reg!(ral::flexspi, flexspi2, AHBRXBUF1CR0,
        PREFETCHEN: 1, PRIORITY: 0, MSTRID: 0, BUFSZ: 64);
    // AHB RX buffers 2/3: all features enabled, max size
    ral::write_reg!(ral::flexspi, flexspi2, AHBRXBUF2CR0,
        PREFETCHEN: 1, PRIORITY: 3, MSTRID: 0xF, BUFSZ: 0xFF);
    ral::write_reg!(ral::flexspi, flexspi2, AHBRXBUF3CR0,
        PREFETCHEN: 1, PRIORITY: 3, MSTRID: 0xF, BUFSZ: 0xFF);

    ral::write_reg!(ral::flexspi, flexspi2, IPRXFCR, CLRIPRXF: 1);
    ral::write_reg!(ral::flexspi, flexspi2, IPTXFCR, CLRIPTXF: 1);
    ral::write_reg!(ral::flexspi, flexspi2, INTEN, 0);

    // FLSHCR1/2 are array registers — use named offsets from RAL field modules.
    // FLSHCR1 A1/A2: TCSH=1 cycle CS hold, TCSS=1 cycle CS setup
    let flshcr1: u32 = (1 << ral::flexspi::FLSHCR1::TCSH::offset)
        | (1 << ral::flexspi::FLSHCR1::TCSS::offset);
    flexspi2.FLSHCR1[0].write(flshcr1);
    flexspi2.FLSHCR1[1].write(flshcr1);
    // FLSHCR2 A1/A2: AWRSEQID=6 (QPI write), ARDSEQID=5 (QPI read)
    let flshcr2: u32 = (6 << ral::flexspi::FLSHCR2::AWRSEQID::offset)
        | (5 << ral::flexspi::FLSHCR2::ARDSEQID::offset);
    flexspi2.FLSHCR2[0].write(flshcr2);
    flexspi2.FLSHCR2[1].write(flshcr2);

    // Enable module
    ral::modify_reg!(ral::flexspi, flexspi2, MCR0, MDIS: 0);

    // ---- LUT programming ----
    ral::write_reg!(ral::flexspi, flexspi2, LUTKEY, KEY: 0x5AF05AF0);
    ral::write_reg!(ral::flexspi, flexspi2, LUTCR, UNLOCK: 1);

    for i in 0..64 {
        flexspi2.LUT[i].write(0);
    }

    // Software reset
    ral::modify_reg!(ral::flexspi, flexspi2, MCR0, SWRESET: 1);
    while ral::read_reg!(ral::flexspi, flexspi2, MCR0, SWRESET) != 0 {}

    ral::write_reg!(ral::flexspi, flexspi2, LUTKEY, KEY: 0x5AF05AF0);
    ral::write_reg!(ral::flexspi, flexspi2, LUTCR, UNLOCK: 1);

    // LUT sequences (instruction encoding, not register fields)
    // Seq 0: Exit QPI mode
    flexspi2.LUT[0].write(lut_word(lut_instr(FLEXSPI_CMD_SDR, FLEXSPI_PADS_4, 0xF5), 0));
    // Seq 1: Reset Enable
    flexspi2.LUT[4].write(lut_word(lut_instr(FLEXSPI_CMD_SDR, FLEXSPI_PADS_1, 0x66), 0));
    // Seq 2: Reset
    flexspi2.LUT[8].write(lut_word(lut_instr(FLEXSPI_CMD_SDR, FLEXSPI_PADS_1, 0x99), 0));
    // Seq 3: Read ID
    flexspi2.LUT[12].write(lut_word(
        lut_instr(FLEXSPI_CMD_SDR, FLEXSPI_PADS_1, 0x9F),
        lut_instr(FLEXSPI_DUMMY_SDR, FLEXSPI_PADS_1, 24),
    ));
    flexspi2.LUT[13].write(lut_word(lut_instr(FLEXSPI_READ_SDR, FLEXSPI_PADS_1, 1), 0));
    // Seq 4: Enter QPI mode
    flexspi2.LUT[16].write(lut_word(lut_instr(FLEXSPI_CMD_SDR, FLEXSPI_PADS_1, 0x35), 0));
    // Seq 5: QPI Fast Read
    flexspi2.LUT[20].write(lut_word(
        lut_instr(FLEXSPI_CMD_SDR, FLEXSPI_PADS_4, 0xEB),
        lut_instr(FLEXSPI_RADDR_SDR, FLEXSPI_PADS_4, 24),
    ));
    flexspi2.LUT[21].write(lut_word(
        lut_instr(FLEXSPI_DUMMY_SDR, FLEXSPI_PADS_4, 6),
        lut_instr(FLEXSPI_READ_SDR, FLEXSPI_PADS_4, 1),
    ));
    // Seq 6: QPI Write
    flexspi2.LUT[24].write(lut_word(
        lut_instr(FLEXSPI_CMD_SDR, FLEXSPI_PADS_4, 0x38),
        lut_instr(FLEXSPI_RADDR_SDR, FLEXSPI_PADS_4, 24),
    ));
    flexspi2.LUT[25].write(lut_word(lut_instr(FLEXSPI_WRITE_SDR, FLEXSPI_PADS_4, 1), 0));

    // ---- PSRAM detection ----
    let size1 = flexspi2_psram_size(&flexspi2, 0);
    if size1 == 0 {
        return 0;
    }

    ral::write_reg!(ral::flexspi, flexspi2, FLSHA1CR0, FLSHSZ: (size1 as u32) * 1024);
    flexspi2_command(&flexspi2, 4, 0);

    let size2 = flexspi2_psram_size(&flexspi2, size1 as u32 * 1024 * 1024);
    if size2 > 0 {
        ral::write_reg!(ral::flexspi, flexspi2, FLSHA2CR0, FLSHSZ: (size2 as u32) * 1024);
        flexspi2_command(&flexspi2, 4, size1 as u32 * 1024 * 1024);
    }

    size1 + size2
}

/// Validate PSRAM by writing and reading back a test pattern.
///
/// Tests `size` bytes starting at [`PSRAM_BASE`]. Returns `true` if
/// all bytes match.
pub fn validate_psram(size: usize) -> bool {
    let base = PSRAM_BASE as *mut u8;
    for i in 0..size {
        unsafe { base.add(i).write_volatile((i & 0xFF) as u8) };
    }
    for i in 0..size {
        let val = unsafe { base.add(i).read_volatile() };
        if val != (i & 0xFF) as u8 {
            return false;
        }
    }
    true
}

/// Get a mutable slice view of PSRAM.
///
/// # Safety
///
/// Caller must ensure:
/// - PSRAM has been initialized via [`init_psram`]
/// - `offset + len` does not exceed the detected PSRAM size
/// - No other code holds an aliasing reference to the same region
pub unsafe fn psram_as_mut_slice(offset: usize, len: usize) -> &'static mut [u8] {
    let ptr = (PSRAM_BASE + offset) as *mut u8;
    core::slice::from_raw_parts_mut(ptr, len)
}
