//! Microchip ATECC608 CryptoAuthLib Integration
//! --------------------------------------------
//! 
//! An example that interacts with the crypto authentication secure elements
//! ATECC608 via the [Rusty driver](`https://crates.io/crates/Rusty_CryptoAuthLib`).
//! Tested with a STM32L4x5 chip connected to ATECC608 over I2C.
//! 
//! The driver library `Rusty_CryptoAuthLib` depends on `defmt`, so some setups are
//! necessary for compilation. Tell cargo to perform custom linking by adding
//! following lines to your `${HOME}/.cargo/config` file or
//! `stm32l4xx-hal/.cargo/config` file.
//! 
//! ```
//! # .cargo/config
//! [target.thumbv7em-none-eabihf]
//! rustflags = [
//!   # likely, there's another `link-arg` flag already there; KEEP it
//!   "-C", "link-arg=-Tdefmt.x", # <- ADD this one
//! ]
//! ```
//! 
//! As defined in `stm32l4xx-hal/.cargo/config`, `cargo run ..` starts semihosting
//! with OpenOCD and GDB. To run this example, provide `openocd.gdb` and
//! `openocd.cfg` files in the same directory as `Cargo.toml`.
//! 
//! ``` bash
//! openocd openocd.cfg &
//! cargo run --example i2c_atecc608_cryptoauthlib \
//!     --target thumbv7em-none-eabihf \
//!     --features stm32l4x5
//! ```
//! 
//! ```
//! # openocd.gdb
//! target extended-remote localhost:3333
//! monitor arm semihosting enable
//! load
//! ```
//! 
//! ```
//! # openocd.cfg
//! source [find interface/stlink-v2.cfg]
//! source [find target/stm32l4x.cfg]
//! ```
#![no_main]
#![no_std]

extern crate panic_semihosting;
use core::fmt::Write;
use cortex_m_rt::entry;
use cortex_m_rt::exception;
use cortex_m_rt::ExceptionFrame;
use cortex_m_semihosting::hio;
use defmt_rtt as _; // global logger
use embedded_hal::timer::CountDown;
use hal::delay::Delay;
use hal::i2c::I2c;
use hal::pac::TIM7;
use hal::prelude::*;
use hal::rcc::{MsiFreq, PllConfig, PllDivider, PllSource};
use hal::timer::Timer;
use nb::Error;
use stm32l4xx_hal as hal;
use void::Void;
use Rusty_CryptoAuthLib::ATECC608A;

#[entry]
fn main() -> ! {
    // Semihosting. Messages will appear in openocd's log output.
    let mut hstdout = hio::hstdout().unwrap();

    // Declare peripherals
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = hal::stm32::Peripherals::take().unwrap();

    // Set up the system clock.
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc
        .cfgr
        // System Clock source = PLL (MSI)
        .pll_source(PllSource::MSI)
        // MSI Frequency(Hz) = 4000000
        .msi(MsiFreq::RANGE4M)
        // SYSCLK(Hz) = 80,000,000, PLL_M = 1, PLL_N = 40, PLL_R = 2
        .sysclk_with_pll(80.mhz(), PllConfig::new(1, 40, PllDivider::Div2))
        .freeze(&mut flash.acr, &mut pwr);

    // Set up SCL & SDA
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);

    let scl = gpiob
        .pb10
        .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper);
    let scl = scl.into_af4(&mut gpiob.moder, &mut gpiob.afrh);

    let sda = gpiob
        .pb11
        .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper);
    let sda = sda.into_af4(&mut gpiob.moder, &mut gpiob.afrh);

    // Construct I2C
    let i2c = I2c::i2c2(dp.I2C2, (scl, sda), 100.khz(), clocks, &mut rcc.apb1r1);

    // Create delay object
    let delay = Delay::new(cp.SYST, clocks);

    // Create timer object
    let timer = TimerWrapper(Timer::tim7(dp.TIM7, 100.khz(), clocks, &mut rcc.apb1r1));

    writeln!(hstdout, "Start testing ATECC608A.");

    // Create CryptoAuth client
    let mut crypto_auth = ATECC608A::new(i2c, delay, timer).unwrap();

    let info = crypto_auth
        .atcab_info()
        .unwrap();
    assert_eq!(info, [0x00, 0x00, 0x60, 0x02]);

    let sha = crypto_auth.atcab_sha(&[0x01, 0x02, 0x03, 0x04, 0x05]).unwrap();
    writeln!(hstdout, "{:02x?}", sha);

    writeln!(hstdout, "ATECC608A test finished.");
    loop {}
}

/// Wraps the `Timer` and bridges the gap between different `CountDown`
/// implementations in `stm32l4xx_hal` and `Rusty_CryptoAuthLib`. The `CountDown`
/// trait requires an associated type of `Time`. In `stm32l4xx_hal` it is `Herz`
/// while `Rusty_CryptoAuthLib` requires `u32`. The wrapper converts `u32` into
/// `Herz` to satisfy the type specification.
struct TimerWrapper(Timer<TIM7>);
impl CountDown for TimerWrapper {
    type Time = u32;

    fn start<T>(&mut self, timeout: T)
    where
        T: Into<u32>,
    {
        // TODO: Confirm the multiplier is correct.
        let timeout = timeout.into().hz();
        self.0.start(timeout)
    }

    fn wait(&mut self) -> Result<(), Error<Void>> {
        self.0.wait()
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
