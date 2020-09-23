//! Inter-Integrated Circuit (I2C) bus

use crate::stm32::{i2c1, I2C1, I2C2};
use cast::u8;
use core::ops::Deref;
use crate::gpio::{Alternate, OpenDrain, Output, AF4};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::rcc::{Clocks, APB1R1};
use crate::time::Hertz;

/// I2C error
#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// NACK
    Nack,
    // Overrun, // slave mode only
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
}

#[doc(hidden)]
mod private {
    pub trait Sealed {}
}

/// SCL pin. This trait is sealed and cannot be implemented.
pub trait SclPin<I2C>: private::Sealed {}

/// SDA pin. This trait is sealed and cannot be implemented.
pub trait SdaPin<I2C>: private::Sealed {}

macro_rules! pins {
    ($spi:ident, $af:ident, SCL: [$($scl:ident),*], SDA: [$($sda:ident),*]) => {
        $(
            impl private::Sealed for $scl<Alternate<$af, Output<OpenDrain>>> {}
            impl SclPin<$spi> for $scl<Alternate<$af, Output<OpenDrain>>> {}
        )*
        $(
            impl private::Sealed for $sda<Alternate<$af, Output<OpenDrain>>> {}
            impl SdaPin<$spi> for $sda<Alternate<$af, Output<OpenDrain>>> {}
        )*
    }
}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident) => {
        loop {
            let isr = $i2c.isr.read();

            if isr.berr().bit_is_set() {
                return Err(Error::Bus);
            } else if isr.arlo().bit_is_set() {
                return Err(Error::Arbitration);
            } else if isr.nackf().bit_is_set() {
                return Err(Error::Nack);
            } else if isr.$flag().bit_is_set() {
                break;
            } else {
                // try again
            }
        }
    };
}

impl <SCL, SDA> I2c<I2C1, (SCL, SDA)> {
    pub fn i2c1<F>(
        i2c: I2C1,
        pins: (SCL, SDA),
        freq: F,
        clocks: Clocks,
        apb1: &mut APB1R1,
    ) -> Self where
        F: Into<Hertz>,
        SCL: SclPin<I2C1>,
        SDA: SdaPin<I2C1>,
    {
        apb1.enr().modify(|_, w| w.i2c1en().set_bit());
        apb1.rstr().modify(|_, w| w.i2c1rst().set_bit());
        apb1.rstr().modify(|_, w| w.i2c1rst().clear_bit());
        Self::new(i2c, pins, freq, clocks)
    }
}

impl <SCL, SDA> I2c<I2C2, (SCL, SDA)> {
    pub fn i2c2<F>(
        i2c: I2C2,
        pins: (SCL, SDA),
        freq: F,
        clocks: Clocks,
        apb1: &mut APB1R1,
    ) -> Self where
        F: Into<Hertz>,
        SCL: SclPin<I2C2>,
        SDA: SdaPin<I2C2>,
    {
        apb1.enr().modify(|_, w| w.i2c2en().set_bit());
        apb1.rstr().modify(|_, w| w.i2c2rst().set_bit());
        apb1.rstr().modify(|_, w| w.i2c2rst().clear_bit());
        Self::new(i2c, pins, freq, clocks)
    }
}

impl<SCL, SDA, I2C> I2c<I2C, (SCL, SDA)> where I2C: Deref<Target = i2c1::RegisterBlock> {
    /// Configures the I2C peripheral to work in master mode
    fn new<F>(
        i2c: I2C,
        pins: (SCL, SDA),
        freq: F,
        clocks: Clocks,
    ) -> Self where
        F: Into<Hertz>,
        SCL: SclPin<I2C>,
        SDA: SdaPin<I2C>,
    {
        let freq = freq.into().0;
        assert!(freq <= 1_000_000);
        // Make sure the I2C unit is disabled so we can configure it
        i2c.cr1.modify(|_, w| w.pe().clear_bit());
        let _ = clocks;
        const STM32CUBEMX_I2CTIMINGTRAITEMENT: u32 = 0x10E09CE6u32;
        const TIMING_CLEAR_MASK: u32 = 0xF0FFFFFFu32;
        // Configure
        i2c.timingr.write(|w| {
            unsafe { w.bits(STM32CUBEMX_I2CTIMINGTRAITEMENT & TIMING_CLEAR_MASK) }
        });
        // Check if the value matches.
        assert_eq!(STM32CUBEMX_I2CTIMINGTRAITEMENT & TIMING_CLEAR_MASK, i2c.timingr.read().bits());
        // Enable the peripheral
        i2c.cr1.write(|w| w.pe().set_bit());

        I2c { i2c, pins }
    }

    /// Releases the I2C peripheral and associated pins
    pub fn free(self) -> (I2C, (SCL, SDA)) {
        (self.i2c, self.pins)
    }
}

impl<PINS, I2C> Write for I2c<I2C, PINS> where I2C: Deref<Target = i2c1::RegisterBlock> {
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // TODO support transfers of more than 255 bytes
        assert!(bytes.len() < 256 && bytes.len() > 0);

        // START and prepare to send `bytes`
        self.i2c.cr2.write(|w| {
            w.sadd()
                .bits((addr as u16) << 1)
                .rd_wrn()
                .clear_bit()
                .nbytes()
                .bits(bytes.len() as u8)
                .start()
                .set_bit()
                .autoend()
                .set_bit()
        });

        for byte in bytes {
            // Wait until we are allowed to send data (START has been ACKed or last byte
            // when through)
            busy_wait!(self.i2c, txis);

            // put byte on the wire
            self.i2c.txdr.write(|w| { w.txdata().bits(*byte) });
        }

        // automatic STOP

        Ok(())
    }
}

impl<PINS, I2C> Read for I2c<I2C, PINS> where I2C: Deref<Target = i2c1::RegisterBlock> {
    type Error = Error;

    fn read(&mut self,
        addr: u8,
        buffer: &mut [u8],) -> Result<(), Error> {
        self.i2c.cr2.write(|w| {
            w.sadd()
                .bits((addr as u16) << 1)
                .rd_wrn()
                .set_bit()
                .nbytes()
                .bits(buffer.len() as u8)
                .start()
                .set_bit()
                .autoend()
                .set_bit()
        });

        for byte in buffer {
            // Wait until we have received something
            busy_wait!(self.i2c, rxne);

            *byte = self.i2c.rxdr.read().rxdata().bits();
        }

        Ok(())
    }
}

impl<PINS, I2C> WriteRead for I2c<I2C, PINS> where I2C: Deref<Target = i2c1::RegisterBlock> {
    type Error = Error;

    fn write_read(
        &mut self,
        addr: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        // TODO support transfers of more than 255 bytes
        assert!(bytes.len() < 256 && bytes.len() > 0);
        assert!(buffer.len() < 256 && buffer.len() > 0);

        // TODO do we have to explicitly wait here if the bus is busy (e.g. another
        // master is communicating)?

        // START and prepare to send `bytes`
        self.i2c.cr2.write(|w| {
            w.sadd()
                .bits((addr as u16) << 1)
                .rd_wrn()
                .clear_bit()
                .nbytes()
                .bits(bytes.len() as u8)
                .start()
                .set_bit()
                .autoend()
                .clear_bit()
        });

        for byte in bytes {
            // Wait until we are allowed to send data (START has been ACKed or last byte
            // when through)
            busy_wait!(self.i2c, txis);

            // put byte on the wire
            self.i2c.txdr.write(|w| { w.txdata().bits(*byte) });
        }

        // Wait until the last transmission is finished
        busy_wait!(self.i2c, tc);

        // reSTART and prepare to receive bytes into `buffer`
        self.i2c.cr2.write(|w| {
            w.sadd()
                .bits((addr as u16) << 1)
                .rd_wrn()
                .set_bit()
                .nbytes()
                .bits(buffer.len() as u8)
                .start()
                .set_bit()
                .autoend()
                .set_bit()
        });

        for byte in buffer {
            // Wait until we have received something
            busy_wait!(self.i2c, rxne);

            *byte = self.i2c.rxdr.read().rxdata().bits();
        }

        // automatic STOP - due to autoend

        Ok(())
    }
}

use crate::gpio::gpioa::{PA10, PA9};
use crate::gpio::gpiob::{PB10, PB11, PB6, PB7};

pins!(I2C1, AF4,
    SCL: [PA9, PB6],
    SDA: [PA10, PB7]);

pins!(I2C2, AF4, SCL: [PB10], SDA: [PB11]);

#[cfg(any(feature = "stm32l4x1", feature = "stm32l4x6"))]
use crate::gpio::gpiob::{PB13, PB14, PB8, PB9};

#[cfg(any(feature = "stm32l4x1", feature = "stm32l4x6"))]
pins!(I2C1, AF4, SCL: [PB8], SDA: [PB9]);

#[cfg(any(feature = "stm32l4x1", feature = "stm32l4x6"))]
pins!(I2C2, AF4, SCL: [PB13], SDA: [PB14]);
