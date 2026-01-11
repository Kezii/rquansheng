//! EEPROM access (UV-K5).
//!
//! This mirrors the reference C firmware implementation:
//! `uv-k5-firmware-custom/driver/eeprom.c` + `driver/i2c.c`.
//!
//! Notes:
//! - The radio uses an external I2C EEPROM accessed via **bit-banged I2C**
//!   on GPIOA pins **PA10=SCL** and **PA11=SDA** (shared with keypad scanning).
//! - Writes are done in fixed 8-byte blocks and include a post-write delay.

use dp32g030 as pac;

use embedded_hal::delay::DelayNs;

use crate::i2c_bitbang::{BitBangI2c, Error as I2cError};

// --- Pin mapping (matches C firmware) ---------------------------------------

// 24xx EEPROM: base 0x50 => 8-bit address 0xA0/0xA1
const EEPROM_I2C_ADDR_WRITE: u8 = 0xA0;
const EEPROM_I2C_ADDR_READ: u8 = 0xA1;

// Size limit used by the reference firmware.
pub const EEPROM_SIZE_BYTES: u16 = 0x2000;

// --- Public EEPROM API ------------------------------------------------------

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Error {
    /// I2C NACK during address/data phase.
    NoAck,
    /// Address out of EEPROM range.
    OutOfRange,
}

/// Read an arbitrary-size buffer from EEPROM.
///
/// Mirrors `EEPROM_ReadBuffer(Address, pBuffer, Size)`.
pub fn read_buffer<D: DelayNs>(delay: &mut D, address: u16, out: &mut [u8]) -> Result<(), Error> {
    if (address as u32) + (out.len() as u32) > (EEPROM_SIZE_BYTES as u32) {
        return Err(Error::OutOfRange);
    }

    let peripherals = unsafe { pac::Peripherals::steal() };
    let syscon = &peripherals.SYSCON;
    let portcon = &peripherals.PORTCON;
    let i2c = BitBangI2c::uvk5_shared_pins();
    i2c.init(syscon, portcon);

    i2c.start(delay);
    i2c.write_byte(delay, portcon, EEPROM_I2C_ADDR_WRITE)
        .map_err(|I2cError::NoAck| Error::NoAck)?;
    i2c.write_byte(delay, portcon, ((address >> 8) & 0xFF) as u8)
        .map_err(|I2cError::NoAck| Error::NoAck)?;
    i2c.write_byte(delay, portcon, (address & 0xFF) as u8)
        .map_err(|I2cError::NoAck| Error::NoAck)?;

    i2c.start(delay);
    i2c.write_byte(delay, portcon, EEPROM_I2C_ADDR_READ)
        .map_err(|I2cError::NoAck| Error::NoAck)?;

    i2c.read_buffer(delay, portcon, out);
    i2c.stop(delay);
    Ok(())
}

/// Write one 8-byte block at `address`, skipping the write if unchanged.
///
/// Mirrors `EEPROM_WriteBuffer(Address, pBuffer)` (which always writes 8 bytes).
pub fn write_buffer_8<D: DelayNs>(
    delay: &mut D,
    address: u16,
    data: &[u8; 8],
) -> Result<(), Error> {
    if address >= EEPROM_SIZE_BYTES {
        return Err(Error::OutOfRange);
    }

    let peripherals = unsafe { pac::Peripherals::steal() };
    let syscon = &peripherals.SYSCON;
    let portcon = &peripherals.PORTCON;
    let i2c = BitBangI2c::uvk5_shared_pins();
    i2c.init(syscon, portcon);

    // Read existing 8 bytes; if identical, avoid EEPROM wear like reference firmware.
    let mut cur = [0u8; 8];
    read_buffer(delay, address, &mut cur)?;
    if &cur == data {
        return Ok(());
    }

    i2c.start(delay);
    i2c.write_byte(delay, portcon, EEPROM_I2C_ADDR_WRITE)
        .map_err(|I2cError::NoAck| Error::NoAck)?;
    i2c.write_byte(delay, portcon, ((address >> 8) & 0xFF) as u8)
        .map_err(|I2cError::NoAck| Error::NoAck)?;
    i2c.write_byte(delay, portcon, (address & 0xFF) as u8)
        .map_err(|I2cError::NoAck| Error::NoAck)?;
    i2c.write_buffer(delay, portcon, data)
        .map_err(|I2cError::NoAck| Error::NoAck)?;
    i2c.stop(delay);

    // Give the EEPROM time to commit (C firmware uses 8ms).
    delay.delay_ms(8);
    Ok(())
}
