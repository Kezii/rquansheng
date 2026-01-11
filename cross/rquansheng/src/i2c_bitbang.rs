//! Bit-banged I2C over GPIOA pins (UV-K5).
//!
//! This mirrors the reference firmware implementation:
//! `uv-k5-firmware-custom/driver/i2c.c`.

use dp32g030 as pac;

use embedded_hal::delay::DelayNs;

use dp30g030_hal::gpio::{FlexPin, Port};

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Error {
    NoAck,
}

/// Bit-banged I2C master using two GPIO pins.
///
/// SDA is switched between output (open-drain) and input for ACK + reads,
/// matching the reference firmware behavior.
#[derive(Copy, Clone)]
pub struct BitBangI2c {
    scl: FlexPin,
    sda: FlexPin,
}

impl BitBangI2c {
    #[inline]
    pub const fn new(scl: FlexPin, sda: FlexPin) -> Self {
        Self { scl, sda }
    }

    /// Convenience for the UV-K5 shared pins: PA10=SCL, PA11=SDA.
    #[inline]
    pub const fn uvk5_shared_pins() -> Self {
        Self::new(FlexPin::new(Port::A, 10), FlexPin::new(Port::A, 11))
    }

    /// Configure pins for I2C: GPIO function, open-drain, output, idle high.
    #[inline]
    pub fn init(&self, syscon: &pac::SYSCON, portcon: &pac::PORTCON) {
        self.scl.configure_gpio(syscon, portcon);
        self.sda.configure_gpio(syscon, portcon);

        self.scl.set_input_enable(portcon, false);
        self.sda.set_input_enable(portcon, false);
        self.scl.set_open_drain(portcon, true);
        self.sda.set_open_drain(portcon, true);
        self.scl.set_output(true);
        self.sda.set_output(true);

        // Idle bus high (released).
        self.scl.write(true);
        self.sda.write(true);
    }

    #[inline(always)]
    fn sda_to_input(&self, portcon: &pac::PORTCON) {
        // C:
        // PORTCON_PORTA_IE |= A11 enable; PORTCON_PORTA_OD &= ~A11; GPIOA->DIR &= ~A11
        self.sda.set_input_enable(portcon, true);
        self.sda.set_open_drain(portcon, false);
        self.sda.set_output(false);
    }

    #[inline(always)]
    fn sda_to_output_open_drain(&self, portcon: &pac::PORTCON) {
        // C:
        // PORTCON_PORTA_IE &= ~A11; PORTCON_PORTA_OD |= A11 enable; GPIOA->DIR |= A11 output
        self.sda.set_input_enable(portcon, false);
        self.sda.set_open_drain(portcon, true);
        self.sda.set_output(true);
    }

    #[inline(always)]
    pub fn start<D: DelayNs>(&self, delay: &mut D) {
        self.sda.write(true);
        delay.delay_us(1);
        self.scl.write(true);
        delay.delay_us(1);
        self.sda.write(false);
        delay.delay_us(1);
        self.scl.write(false);
        delay.delay_us(1);
    }

    #[inline(always)]
    pub fn stop<D: DelayNs>(&self, delay: &mut D) {
        self.sda.write(false);
        delay.delay_us(1);
        self.scl.write(false);
        delay.delay_us(1);
        self.scl.write(true);
        delay.delay_us(1);
        self.sda.write(true);
        delay.delay_us(1);
    }

    #[inline(always)]
    pub fn write_byte<D: DelayNs>(
        &self,
        delay: &mut D,
        portcon: &pac::PORTCON,
        mut data: u8,
    ) -> Result<(), Error> {
        self.scl.write(false);
        delay.delay_us(1);

        for _ in 0..8 {
            self.sda.write((data & 0x80) != 0);
            data <<= 1;
            delay.delay_us(1);
            self.scl.write(true);
            delay.delay_us(1);
            self.scl.write(false);
            delay.delay_us(1);
        }

        // ACK phase: sample SDA while SCL is high.
        self.sda_to_input(portcon);
        self.sda.write(true); // release
        delay.delay_us(1);
        self.scl.write(true);
        delay.delay_us(1);

        // Reference firmware spins up to 255 iterations waiting for SDA to go low.
        let mut ack = false;
        for _ in 0..255u16 {
            if !self.sda.read() {
                ack = true;
                break;
            }
        }

        self.scl.write(false);
        delay.delay_us(1);
        self.sda_to_output_open_drain(portcon);
        self.sda.write(true);

        if ack {
            Ok(())
        } else {
            Err(Error::NoAck)
        }
    }

    #[inline(always)]
    pub fn read_byte<D: DelayNs>(
        &self,
        delay: &mut D,
        portcon: &pac::PORTCON,
        final_byte: bool,
    ) -> u8 {
        self.sda_to_input(portcon);

        let mut data: u8 = 0;
        for _ in 0..8 {
            self.scl.write(false);
            delay.delay_us(1);
            self.scl.write(true);
            delay.delay_us(1);
            data <<= 1;
            delay.delay_us(1);
            if self.sda.read() {
                data |= 1;
            }
            self.scl.write(false);
            delay.delay_us(1);
        }

        // Send ACK (SDA low) or NACK (SDA high) after byte.
        self.sda_to_output_open_drain(portcon);
        self.scl.write(false);
        delay.delay_us(1);
        self.sda.write(final_byte);
        delay.delay_us(1);
        self.scl.write(true);
        delay.delay_us(1);
        self.scl.write(false);
        delay.delay_us(1);

        data
    }

    #[inline(always)]
    pub fn read_buffer<D: DelayNs>(&self, delay: &mut D, portcon: &pac::PORTCON, out: &mut [u8]) {
        if out.is_empty() {
            return;
        }
        let last = out.len() - 1;
        for b in &mut out[..last] {
            delay.delay_us(1);
            *b = self.read_byte(delay, portcon, false);
        }
        delay.delay_us(1);
        out[last] = self.read_byte(delay, portcon, true);
    }

    #[inline(always)]
    pub fn write_buffer<D: DelayNs>(
        &self,
        delay: &mut D,
        portcon: &pac::PORTCON,
        data: &[u8],
    ) -> Result<(), Error> {
        for &b in data {
            self.write_byte(delay, portcon, b)?;
        }
        Ok(())
    }
}
