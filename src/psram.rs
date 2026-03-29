use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::board::PsramToken;

/// Raw PSRAM-backed storage placed in the `.psram.data` linker section.
///
/// This type is not intended for direct use. It is created by the
/// [`psram_static!`](crate::psram_static) macro and accessed through
/// [`PsramStatic`].
#[repr(transparent)]
#[doc(hidden)]
pub struct PsramData<T>(MaybeUninit<T>);

// SAFETY: PsramData is only accessed through PsramStatic, which
// enforces one-shot exclusive access via AtomicBool.
unsafe impl<T: Send> Sync for PsramData<T> {}

impl<T> PsramData<T> {
    /// Create a new `PsramData` with an initial value.
    ///
    /// The value becomes the ELF content for this static. When placed
    /// in `.psram.data`, the linker stores it at the LMA (flash).
    /// `initialize_psram` copies it to the VMA (PSRAM) at runtime.
    #[doc(hidden)]
    pub const fn __new(val: T) -> Self {
        Self(MaybeUninit::new(val))
    }
}

/// Handle to a PSRAM-backed static variable.
///
/// Created by the [`psram_static!`](crate::psram_static) macro.
/// This type lives in normal RAM (DTCM) and holds a pointer to the
/// actual data in PSRAM plus a one-shot flag.
///
/// Call [`.take(token)`](PsramStatic::take) to obtain a one-shot
/// `&'static mut T` reference, following the `static_cell` pattern.
///
/// The `AtomicBool` flag is kept in DTCM (not PSRAM) to avoid
/// relying on exclusive-monitor (LDREX/STREX) support for
/// FlexSPI2-mapped external memory.
pub struct PsramStatic<T> {
    data: *const PsramData<T>,
    taken: AtomicBool,
}

// SAFETY: PsramStatic is Sync because:
// - `taken` is AtomicBool (inherently Sync).
// - `data` is a raw pointer to a static in .psram.data; the one-shot
//   take() ensures no aliasing. T: Send is required so the data can
//   be safely shared across contexts.
unsafe impl<T: Send> Sync for PsramStatic<T> {}
unsafe impl<T: Send> Send for PsramStatic<T> {}

impl<T> PsramStatic<T> {
    /// Create a new `PsramStatic` pointing to PSRAM-backed data.
    ///
    /// # Safety
    ///
    /// `data` must point to a static in the `.psram.data` section whose
    /// initializer will be copied from flash by `initialize_psram`.
    /// This is handled by the `psram_static!` macro.
    #[doc(hidden)]
    pub const unsafe fn __new(data: *const PsramData<T>) -> Self {
        Self {
            data,
            taken: AtomicBool::new(false),
        }
    }

    /// Take a one-shot `&'static mut T` reference to the PSRAM-backed value.
    ///
    /// Returns `Some` on the first call, `None` on subsequent calls.
    /// The `PsramToken` proves that PSRAM has been initialized and
    /// the initializer data has been copied from flash.
    ///
    /// The caller decides how to use the mutable reference:
    /// - Keep it for exclusive access.
    /// - Reborrow as `&'static T` for shared access (one-way downgrade).
    /// - Wrap in `Mutex<RefCell<...>>` for shared interior mutability.
    pub fn take(&self, _token: PsramToken) -> Option<&'static mut T> {
        if self
            .taken
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
        {
            // SAFETY:
            // - PsramToken guarantees initialize_psram completed, so the
            //   data has been copied from flash into PSRAM and is initialized.
            // - compare_exchange succeeded, so this is the first and only
            //   call, guaranteeing no aliasing.
            // - The 'static lifetime is sound because the underlying
            //   storage is a true static in the .psram.data section.
            // - PsramData<T> is repr(transparent) over MaybeUninit<T>,
            //   which has the same size and alignment as initialized T.
            Some(unsafe { &mut *(self.data as *mut T) })
        } else {
            None
        }
    }
}

/// Declare typed static variables backed by PSRAM.
///
/// Each static declared in this macro has its data placed in the
/// `.psram.data` linker section (memory-mapped PSRAM at `0x7000_0000`),
/// with initializer data stored in flash. A handle with the one-shot
/// flag lives in normal RAM (DTCM). Call
/// [`initialize_psram`](crate::board::initialize_psram) to copy
/// initializers from flash into PSRAM before accessing any static.
///
/// Access follows the `static_cell` pattern: call `.take(token)` to
/// get a one-shot `&'static mut T`. The first call returns `Some`;
/// subsequent calls return `None`.
///
/// # Example
///
/// ```no_run
/// use teensy4_bsp as bsp;
/// use bsp::{board, psram_static};
///
/// psram_static! {
///     static BUFFER: [u8; 1024] = [0u8; 1024];
/// }
///
/// # #[bsp::rt::entry]
/// # fn main() -> ! {
/// let mut resources = board::t41(board::instances());
/// let token = board::initialize_psram(
///     resources.flexspi2,
///     &mut resources.iomuxc,
///     Default::default(),
/// )
/// .unwrap()
/// .token();
///
/// let buf: &'static mut [u8; 1024] = BUFFER.take(token).unwrap();
/// # loop {}
/// # }
/// ```
///
/// # Flash cost
///
/// Initializer data is stored in flash. Large initializers (e.g.,
/// `[0u8; 1_048_576]`) consume corresponding flash space. There is
/// no `.psram.bss` zero-fill optimization.
#[cfg(feature = "psram")]
#[macro_export]
macro_rules! psram_static {
    ($(static $name:ident : $ty:ty = $init:expr;)*) => {
        $(
            // Nested-static trick: Rust allows `static` items inside a
            // block expression that initializes another `static`. The
            // inner PSRAM_DATA gets its own fixed address and the
            // #[link_section] places it in PSRAM. The outer static has
            // no link_section, so it lands in .bss (DTCM) — keeping the
            // AtomicBool in memory that supports LDREX/STREX. Each
            // macro repetition gets its own block scope, so multiple
            // declarations each produce an independent PSRAM_DATA with
            // no name collision.
            static $name: $crate::PsramStatic<$ty> = {
                #[link_section = ".psram.data"]
                static PSRAM_DATA: $crate::PsramData<$ty> =
                    $crate::PsramData::__new($init);

                // SAFETY: PSRAM_DATA is in .psram.data and its
                // initializer is copied from flash by initialize_psram
                // before any PsramToken can exist.
                unsafe {
                    $crate::PsramStatic::__new(
                        &PSRAM_DATA as *const $crate::PsramData<$ty>
                    )
                }
            };
        )*
    };
}
