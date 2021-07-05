//! Flash memory module
//!
//! Example usage of flash programming interface:
//!
//! ```
//! fn program_region(mut flash: flash::McuFlash) -> Result<(), flash::Error> {
//!     // Unlock the flashing module
//!     let mut prog = flash.unlock()?;
//!     let (start, end) = prog.range();
//!
//!     let page = start + 1024;
//!
//!     // Perform the erase and programing operation
//!     let data = [
//!         0x1111_1112_1113_1114,
//!         0x2221_2222_2223_2224,
//!         0x3331_3332_3333_3334,
//!     ];
//!     prog.try_write(page, &data)?;
//!
//!     // Check result (not needed, but done for this example)
//!     let addr = page as *const u64;
//!     assert!(unsafe { core::ptr::read(addr) } == data[0]);
//!     assert!(unsafe { core::ptr::read(addr.offset(1)) } == data[1]);
//!     assert!(unsafe { core::ptr::read(addr.offset(2)) } == data[2]);
//!
//!     Ok(())
//! }
//! ```

#![deny(missing_docs)]

use crate::stm32::{flash, FLASH};
use core::convert::TryInto;
use core::{mem, ops::Drop, ptr};
use embedded_storage::{
    nor_flash::{NorFlash, ReadNorFlash},
    Region,
};

#[derive(Clone, Copy, Debug)]
/// Size of the MCU flash
pub enum FlashVariant {
    /// 1MB flash size
    #[cfg(any(feature = "stm32l4x1", feature = "stm32l4x5", feature = "stm32l4x6"))]
    Size1024KB = 1024,
    /// 512KB flash size
    #[cfg(any(
        feature = "stm32l4x1",
        feature = "stm32l4x2",
        feature = "stm32l4x5",
        feature = "stm32l4x6"
    ))]
    Size512KB = 512,
    /// 256KB flash size
    #[cfg(any(
        feature = "stm32l4x1",
        feature = "stm32l4x2",
        feature = "stm32l4x3",
        feature = "stm32l4x5",
        feature = "stm32l4x6"
    ))]
    Size256KB = 256,
    #[cfg(any(feature = "stm32l4x1", feature = "stm32l4x2", feature = "stm32l4x3",))]
    Size128KB = 128,
    #[cfg(feature = "stm32l4x2")]
    Size64KB = 64,
}

impl FlashVariant {
    const fn bytes(self) -> usize {
        self as usize * 1024
    }
}

/// Error type of flash peripheral when programming
/// If an error occurs during a program or erase operation, one of the following error flags is set
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ProgrammingError {
    /// Programming Error
    ///
    /// In standard programming: PROGERR is set if the word to write is not previously erased
    /// (except if the value to program is full zero).
    Programming,
    /// Size Programming Error
    ///
    /// In standard programming or in fast programming: only double word can be
    /// programmed and only 32-bit data can be written. SIZERR is set if a byte or an halfword is written
    Size,
    /// Alignment Programming error
    ///
    /// PGAERR is set if one of the following conditions occurs:
    /// – In standard programming: the first word to be programmed is not aligned with a
    ///   double word address, or the second word doesn’t belong to the same double word
    ///   address.
    /// – In fast programming: the data to program doesn’t belong to the same row than the
    ///   previous programmed double words, or the address to program is not greater than
    ///   the previous one.
    Alignment,
    /// Programming Sequence Error
    ///
    /// PGSERR is set if one of the following conditions occurs:
    /// – In the standard programming sequence or the fast programming sequence: a data
    ///   is written when PG and FSTPG are cleared.
    /// – In the standard programming sequence or the fast programming sequence:
    ///   MER1, MER2, and PER are not cleared when PG or FSTPG is set.
    /// – In the fast programming sequence: the Mass erase is not performed before setting
    ///   FSTPG bit.
    /// – In the mass erase sequence: PG, FSTPG, and PER are not cleared when MER1
    ///   or MER2 is set.
    /// – In the page erase sequence: PG, FSTPG, MER1 and MER2 are not cleared when
    ///   PER is set.
    /// – PGSERR is set also if PROGERR, SIZERR, PGAERR, MISSERR, FASTERR or
    ///   PGSERR is set due to a previous programming error
    Sequence,
    /// Write Protection Error
    ///
    /// WRPERR is set if one of the following conditions occurs:
    /// – Attempt to program or erase in a write protected area (WRP) or in a PCROP area.
    /// – Attempt to perform a bank erase when one page or more is protected by WRP or
    ///   PCROP.
    /// – The debug features are connected or the boot is executed from SRAM or from
    ///   System flash when the read protection (RDP) is set to Level 1.
    /// – Attempt to modify the option bytes when the read protection (RDP) is set to
    ///   Level 2.
    WriteProtection,
    /// Fast Programming Data Miss Error
    ///
    /// In fast programming: all the data must be written successively. MISSERR is set if the
    /// previous data programmation is finished and the next data to program is not written yet
    DataMiss,
    /// Fast Programming Error
    ///
    /// In fast programming: FASTERR is set if one of the following conditions occurs:
    /// – When FSTPG bit is set for more than 7ms which generates a time-out detection.
    /// – When the row fast programming has been interrupted by a MISSERR, PGAERR,
    ///   WRPERR or SIZERR.
    FastProgramming,
}

/// Error type of flash peripheral
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Error {
    /// Flash controller is not done yet
    Busy,
    /// Error detected (by command execution, or because no command could be executed)
    Programming(ProgrammingError),
    /// Set during read if ECC decoding logic detects correctable or uncorrectable error
    EccError,
    /// Page number is out of range
    OutOfBounds,
    /// (Legal) command failed
    Failure,
}

/// Extension trait to constrain the FLASH peripheral
pub trait FlashExt {
    /// Constrains the FLASH peripheral to play nicely with the other abstractions
    fn constrain(self, variant: FlashVariant) -> McuFlash;
}

impl FlashExt for FLASH {
    fn constrain(self, variant: FlashVariant) -> McuFlash {
        McuFlash {
            acr: ACR {},
            pdkeyr: PDKEYR {},
            keyr: KEYR {},
            optkeyr: OPTKEYR {},
            sr: SR {},
            cr: CR {},
            eccr: ECCR {},
            pcrop1sr: PCROP1SR {},
            pcrop1er: PCROP1ER {},
            wrp1ar: WRP1AR {},
            wrp1br: WRP1BR {},
            memory_map: MemoryMap(variant),
        }
    }
}

/// Constrained FLASH peripheral
pub struct McuFlash {
    /// Opaque ACR register
    pub acr: ACR,
    /// Opaque PDKEYR register
    pub pdkeyr: PDKEYR,
    /// Opaque KEYR register
    pub keyr: KEYR,
    /// Opaque OPTKEYR register
    pub optkeyr: OPTKEYR,
    /// Opaque SR register
    pub sr: SR,
    /// Opaque SR register
    pub cr: CR,
    /// Opaque ECCR register
    pub eccr: ECCR,
    /// Opaque PCROP1SR register
    pub pcrop1sr: PCROP1SR,
    /// Opaque PCROP1ER register
    pub pcrop1er: PCROP1ER,
    /// Opaque WRP1AR register
    pub wrp1ar: WRP1AR,
    /// Opaque WRP1BR register
    pub wrp1br: WRP1BR,
    /// Memory map of the given Flash variant
    memory_map: MemoryMap,
}

macro_rules! generate_register {
    ($a:ident, $b:ident, $name:expr) => {
        #[doc = "Opaque "]
        #[doc = $name]
        #[doc = " register"]
        pub struct $a;

        impl $a {
            #[allow(unused)]
            pub(crate) fn $b(&mut self) -> &flash::$a {
                // NOTE(unsafe) this proxy grants exclusive access to this register
                unsafe { &(*FLASH::ptr()).$b }
            }
        }
    };

    ($a:ident, $b:ident) => {
        generate_register!($a, $b, stringify!($a));
    };
}

generate_register!(ACR, acr);
generate_register!(PDKEYR, pdkeyr);
generate_register!(KEYR, keyr);
generate_register!(OPTKEYR, optkeyr);
generate_register!(SR, sr);
generate_register!(CR, cr);
generate_register!(ECCR, eccr);
generate_register!(PCROP1SR, pcrop1sr);
generate_register!(PCROP1ER, pcrop1er);
generate_register!(WRP1AR, wrp1ar);
generate_register!(WRP1BR, wrp1br);

const FLASH_KEY1: u32 = 0x4567_0123;
const FLASH_KEY2: u32 = 0xCDEF_89AB;

impl McuFlash {
    /// Unlock the flash registers via KEYR to access the flash programming
    pub fn unlock<'a>(&'a mut self) -> Result<FlashProgramming<'a>, Error> {
        let keyr = self.keyr.keyr();
        unsafe {
            keyr.write(|w| w.bits(FLASH_KEY1));
            keyr.write(|w| w.bits(FLASH_KEY2));
        }

        let cr = &mut self.cr;
        if cr.cr().read().lock().bit_is_clear() {
            let sr = &mut self.sr;
            Ok(FlashProgramming {
                sr,
                cr,
                memory_map: self.memory_map,
            })
        } else {
            Err(Error::Failure)
        }
    }

    /// Return the capacity of the flash in bytes
    pub fn capacity(&self) -> usize {
        self.memory_map.0.bytes()
    }
}

/// Flash programming interface
pub struct FlashProgramming<'a> {
    sr: &'a mut SR,
    cr: &'a mut CR,
    memory_map: MemoryMap,
}

impl<'a> FlashProgramming<'a> {
    fn status(&mut self) -> Result<(), Error> {
        let sr = self.sr.sr().read();

        if sr.bsy().bit_is_set() {
            Err(Error::Busy)
        } else if sr.pgaerr().bit_is_set() {
            Err(Error::Programming(ProgrammingError::Alignment))
        } else if sr.progerr().bit_is_set() {
            Err(Error::Programming(ProgrammingError::Programming))
        } else if sr.wrperr().bit_is_set() {
            Err(Error::Programming(ProgrammingError::WriteProtection))
        } else if sr.sizerr().bit_is_set() {
            Err(Error::Programming(ProgrammingError::Size))
        } else if sr.pgserr().bit_is_set() {
            Err(Error::Programming(ProgrammingError::Sequence))
        } else {
            Ok(())
        }
    }

    /// Lock the flash memory controller
    fn lock(&mut self) {
        self.cr.cr().modify(|_, w| w.lock().set_bit());
    }

    /// Wait till last flash operation is complete
    fn wait(&mut self) -> Result<(), Error> {
        while self.sr.sr().read().bsy().bit_is_set() {}
        self.status()
    }

    /// Erase all flash pages, note that this will erase the current running program if it is not
    /// called from a program running in RAM.
    pub fn erase_banks(&mut self, banks: &[MemoryBank]) -> Result<(), Error> {
        self.status()?;

        // Set the MER1 bit or/and MER2 (depending on the bank) in the Flash control register
        // (FLASH_CR). Both banks can be selected in the same operation.
        self.cr.cr().modify(|_, w| {
            for bank in banks {
                match bank {
                    MemoryBank::Bank1 => w.mer1().set_bit(),
                    MemoryBank::Bank2 => w.mer2().set_bit(),
                };
            }
            w
        });

        // Set the STRT bit in the FLASH_CR register.
        self.cr.cr().modify(|_, w| w.start().set_bit());

        // Wait for the BSY bit to be cleared in the Flash status register (FLASH_SR)
        let res = self.wait();

        self.cr.cr().modify(|_, w| {
            for bank in banks {
                match bank {
                    MemoryBank::Bank1 => w.mer1().clear_bit(),
                    MemoryBank::Bank2 => w.mer2().clear_bit(),
                };
            }
            w
        });

        res
    }

    fn erase_page(&mut self, page: &Page) -> Result<(), Error> {
        self.status()?;

        let page_number = (page.location - MemoryMap::start()) / page.size as u32;
        match page.area {
            Area::Main(MemoryBank::Bank1) => {
                self.cr.cr().modify(|_, w| unsafe {
                    w.bker()
                        .clear_bit()
                        .pnb()
                        .bits(page_number as u8)
                        .per()
                        .set_bit()
                });
            }
            Area::Main(MemoryBank::Bank2) => {
                self.cr.cr().modify(|_, w| unsafe {
                    w.bker()
                        .set_bit()
                        .pnb()
                        .bits((page_number - 256) as u8)
                        .per()
                        .set_bit()
                });
            }
            // TODO: Handle Area::SystemMemory, Area::OneTimeProgrammable & Area::OptionBytes
            _ => {
                return Err(Error::OutOfBounds);
            }
        }

        self.cr.cr().modify(|_, w| w.start().set_bit());

        let res = self.wait();

        self.cr.cr().modify(|_, w| w.per().clear_bit());

        res
    }

    /// Programming in a previously programmed address is not allowed except if the data to write
    /// is full zero, and any attempt will set PROGERR flag in the Flash status register
    /// (FLASH_SR).
    ///
    /// It is only possible to program double word (2 x 32-bit data).
    /// • Any attempt to write byte or half-word will set SIZERR flag in the FLASH_SR register.
    /// • Any attempt to write a double word which is not aligned with a double word address will
    ///   set PGAERR flag in the FLASH_SR register.
    fn write_native(&mut self, data: &[u64], aligned_address: usize) -> Result<(), Error> {
        self.status()?;
        // NB: The check for alignment of the address, and that the flash is erased is made by the
        // flash controller. The `wait` function will return the proper error codes.
        let mut address = aligned_address as *mut u32;

        self.cr.cr().modify(|_, w| w.pg().set_bit());

        for dword in data {
            unsafe {
                ptr::write_volatile(address, *dword as u32);
                ptr::write_volatile(address.add(1), (*dword >> 32) as u32);

                address = address.add(2);
            }

            self.wait()?;

            if self.sr.sr().read().eop().bit_is_set() {
                self.sr.sr().modify(|_, w| w.eop().clear_bit());
            }
        }

        self.cr.cr().modify(|_, w| w.pg().clear_bit());

        Ok(())
    }

    fn write_bytes(&mut self, data: &[u8], address: u32) -> Result<(), Error> {
        let address = address as usize;
        let address_offset = address % mem::align_of::<u64>();
        let unaligned_size = (mem::size_of::<u64>() - address_offset) % mem::size_of::<u64>();

        if unaligned_size > 0 {
            let unaligned_data = &data[..unaligned_size];
            // Handle unaligned address data, make it into a native write
            let mut dword = 0xffff_ffff_ffff_ffffu64;
            for b in unaligned_data {
                dword = (dword >> 8) | ((*b as u64) << 56);
            }

            let unaligned_address = address - address_offset;
            let native = &[dword];
            self.write_native(native, unaligned_address)?;
        }

        // Handle aligned address data
        let aligned_data = &data[unaligned_size..];
        let mut aligned_address = if unaligned_size > 0 {
            address - address_offset + mem::size_of::<u64>()
        } else {
            address
        };

        let mut chunks = aligned_data.chunks_exact(mem::size_of::<u64>());

        while let Some(exact_chunk) = chunks.next() {
            // Write chunks
            let native = &[u64::from_ne_bytes(exact_chunk.try_into().unwrap())];
            self.write_native(native, aligned_address)?;
            aligned_address += mem::size_of::<u64>();
        }

        let rem = chunks.remainder();

        if !rem.is_empty() {
            let mut dword = 0xffff_ffff_ffff_ffffu64;
            // Write remainder
            for b in rem.iter().rev() {
                dword = (dword << 8) | *b as u64;
            }

            let native = &[dword];
            self.write_native(native, aligned_address)?;
        }

        Ok(())
    }
}

impl<'a> Drop for FlashProgramming<'a> {
    fn drop(&mut self) {
        // Lock on drop
        self.lock();
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
/// Describes which flash area is being addressed
enum Area {
    /// Main flash area
    Main(MemoryBank),
    /// System memory flash area
    SystemMemory,
    /// OTA (One time programmable) flash area
    OneTimeProgrammable,
    /// Option bytes flash area
    OptionBytes,
}

#[derive(Copy, Clone, Debug, PartialEq)]
/// Describes which memory bank is being addressed
pub enum MemoryBank {
    /// Memory bank 1
    Bank1,
    /// Memory bank 2
    Bank2,
}

#[derive(Copy, Clone, Debug, PartialEq)]
/// Flash page, as being the smallest entity to erase
struct Page {
    area: Area,
    location: u32,
    size: usize,
}

const MAX_PAGE_SIZE: usize = 2048;

#[derive(Clone, Copy)]
/// Memory map describing the sections and pages of the flash
pub struct MemoryMap(FlashVariant);

impl MemoryMap {
    /// Get an iterator over all the possible pages of the flash memory
    fn pages(&self) -> impl Iterator<Item = Page> {
        let page_size = self.page_size();
        let last_page = self.0.bytes() / page_size;

        (0..last_page).map(move |page_number| {
            let bank = if page_number < (last_page+1) / 2 {
                MemoryBank::Bank1
            } else {
                MemoryBank::Bank2
            };

            Page::new(
                Area::Main(bank),
                Self::start() + (page_number * page_size) as u32,
                page_size as usize,
            )
        })
    }

    /// The base address (start) of the flash memory
    pub const fn start() -> u32 {
        0x0800_0000
    }

    /// The end address of the flash memory
    pub const fn end(&self) -> u32 {
        (0x0800_0000 + self.0.bytes() - 1) as u32
    }

    fn page_size(&self) -> usize {
        MAX_PAGE_SIZE
    }
}

impl Region for Page {
    fn contains(&self, address: u32) -> bool {
        (self.location <= address) && (address <= self.end())
    }
}

impl Page {
    /// Construct a new `Page`
    const fn new(area: Area, location: u32, size: usize) -> Self {
        Self {
            area,
            location,
            size,
        }
    }

    /// The end address of the page
    const fn end(&self) -> u32 {
        self.location + self.size as u32 - 1
    }
}

impl<'a> ReadNorFlash for FlashProgramming<'a> {
    type Error = Error;

    const READ_SIZE: usize = 1;

    fn try_read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        if offset > self.capacity() as u32 {
            return Err(Error::OutOfBounds);
        }

        let mut address = (MemoryMap::start() + offset) as *const u8;

        for data in bytes {
            unsafe {
                *data = ptr::read(address);
                address = address.add(1);
            }
        }
        Ok(())
    }

    fn capacity(&self) -> usize {
        self.memory_map.0.bytes()
    }
}

impl<'a> NorFlash for FlashProgramming<'a> {
    const WRITE_SIZE: usize = 8;
    const ERASE_SIZE: usize = 2048;

    fn try_write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if offset as usize + bytes.len() > self.capacity() {
            return Err(Error::OutOfBounds);
        }

        if offset % Self::WRITE_SIZE as u32 != 0 {
            return Err(Error::Programming(ProgrammingError::Alignment));
        }

        if bytes.len() % Self::WRITE_SIZE != 0 {
            return Err(Error::Programming(ProgrammingError::Size));
        }

        let mut aligned_address = (MemoryMap::start() + offset) as usize;
        for exact_chunk in bytes.chunks_exact(Self::WRITE_SIZE) {
            let native = &[u64::from_ne_bytes(exact_chunk.try_into().unwrap())];
            self.write_native(native, aligned_address)?;
            aligned_address += Self::WRITE_SIZE;
        }

        Ok(())
    }

    fn try_erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        // Check that from & to is properly aligned to a proper erase resolution
        if to % 2048 != 0 || from % 2048 != 0 {
            return Err(Error::Programming(ProgrammingError::Alignment));
        }

        self.memory_map
            .pages()
            .skip_while(|page| !page.contains(MemoryMap::start() + from))
            .take_while(|page| !page.contains(MemoryMap::start() + to))
            .try_for_each(|page| self.erase_page(&page))
    }
}
