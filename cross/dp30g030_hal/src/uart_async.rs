//! Async UART IO helpers for DP32G030.
//!
//! This module adds `embedded-io-async` trait implementations for the UART types
//! defined in [`crate::uart`].
//!
//! Notes:
//! - This is a minimal implementation based on polling UART flags.
//! - It does not configure interrupts or provide waker integration; the futures
//!   simply re-wake themselves when they need to wait.

use core::future::poll_fn;
use core::task::Poll;

use dp32g030 as pac;

use embedded_io_async as eioa;

use crate::uart::{Error, Instance, Rx, Tx, Uart};

// `embedded-io-async` re-exports `embedded-io` v0.7's Error traits. Our `Error`
// type already implements embedded-io v0.6's traits in `uart.rs`, so we provide
// the v0.7 impl here as well.
impl eioa::Error for Error {
    #[inline]
    fn kind(&self) -> eioa::ErrorKind {
        match self {
            Error::RxTimeout => eioa::ErrorKind::TimedOut,
            Error::BadConfig => eioa::ErrorKind::InvalidInput,
            Error::Overrun | Error::Parity | Error::Frame => eioa::ErrorKind::Other,
        }
    }
}

macro_rules! impl_uart_async {
    ($UART:ty,
     $tdr:ident, $rdr:ident, $if_:ident) => {
        impl<TX, RX> eioa::ErrorType for Uart<$UART, TX, RX> {
            type Error = Error;
        }

        impl<TX> eioa::ErrorType for Tx<$UART, TX> {
            type Error = Error;
        }

        impl<RX> eioa::ErrorType for Rx<$UART, RX> {
            type Error = Error;
        }

        impl<TX, RX> eioa::Read for Uart<$UART, TX, RX> {
            async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
                if buf.is_empty() {
                    return Ok(0);
                }

                let regs = <$UART as Instance>::regs();

                #[inline(always)]
                fn check_and_clear_rx_errors(regs: &<$UART as Instance>::Regs) -> Result<(), Error> {
                    let ifr = regs.$if_().read();

                    // Latched error flags (write-1-to-clear)
                    if ifr.rxfifo_ovf().bit_is_set() {
                        regs.$if_().write(|w| unsafe { w.bits(1u32 << 8) });
                        return Err(Error::Overrun);
                    }
                    if ifr.parritye().bit_is_set() {
                        regs.$if_().write(|w| unsafe { w.bits(1u32 << 3) });
                        return Err(Error::Parity);
                    }
                    if ifr.stope().bit_is_set() {
                        regs.$if_().write(|w| unsafe { w.bits(1u32 << 4) });
                        return Err(Error::Frame);
                    }
                    if ifr.rxto().bit_is_set() {
                        regs.$if_().write(|w| unsafe { w.bits(1u32 << 5) });
                        return Err(Error::RxTimeout);
                    }

                    Ok(())
                }

                // Wait until at least one byte is available (or an error happens).
                poll_fn(|cx| {
                    if let Err(e) = check_and_clear_rx_errors(regs) {
                        return Poll::Ready(Err(e));
                    }

                    let ifr = regs.$if_().read();
                    if ifr.rxfifo_empty().bit_is_clear() {
                        Poll::Ready(Ok(()))
                    } else {
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                })
                .await?;

                // Read what we can without waiting further.
                let mut n = 0usize;
                while n < buf.len() {
                    check_and_clear_rx_errors(regs)?;

                    let ifr = regs.$if_().read();
                    if ifr.rxfifo_empty().bit_is_set() {
                        break;
                    }

                    let data = regs.$rdr().read().rdr().bits() as u16;
                    buf[n] = (data & 0xFF) as u8;
                    n += 1;
                }

                Ok(n)
            }
        }

        impl<RX> eioa::Read for Rx<$UART, RX> {
            async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
                if buf.is_empty() {
                    return Ok(0);
                }

                let regs = <$UART as Instance>::regs();

                #[inline(always)]
                fn check_and_clear_rx_errors(regs: &<$UART as Instance>::Regs) -> Result<(), Error> {
                    let ifr = regs.$if_().read();

                    // Latched error flags (write-1-to-clear)
                    if ifr.rxfifo_ovf().bit_is_set() {
                        regs.$if_().write(|w| unsafe { w.bits(1u32 << 8) });
                        return Err(Error::Overrun);
                    }
                    if ifr.parritye().bit_is_set() {
                        regs.$if_().write(|w| unsafe { w.bits(1u32 << 3) });
                        return Err(Error::Parity);
                    }
                    if ifr.stope().bit_is_set() {
                        regs.$if_().write(|w| unsafe { w.bits(1u32 << 4) });
                        return Err(Error::Frame);
                    }
                    if ifr.rxto().bit_is_set() {
                        regs.$if_().write(|w| unsafe { w.bits(1u32 << 5) });
                        return Err(Error::RxTimeout);
                    }

                    Ok(())
                }

                poll_fn(|cx| {
                    if let Err(e) = check_and_clear_rx_errors(regs) {
                        return Poll::Ready(Err(e));
                    }

                    let ifr = regs.$if_().read();
                    if ifr.rxfifo_empty().bit_is_clear() {
                        Poll::Ready(Ok(()))
                    } else {
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                })
                .await?;

                let mut n = 0usize;
                while n < buf.len() {
                    check_and_clear_rx_errors(regs)?;

                    let ifr = regs.$if_().read();
                    if ifr.rxfifo_empty().bit_is_set() {
                        break;
                    }

                    let data = regs.$rdr().read().rdr().bits() as u16;
                    buf[n] = (data & 0xFF) as u8;
                    n += 1;
                }

                Ok(n)
            }
        }

        impl<TX, RX> eioa::Write for Uart<$UART, TX, RX> {
            async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                if buf.is_empty() {
                    return Ok(0);
                }

                let regs = <$UART as Instance>::regs();

                // Wait until we can write at least one byte.
                poll_fn(|cx| {
                    let ifr = regs.$if_().read();
                    if ifr.txfifo_full().bit_is_clear() {
                        Poll::Ready(())
                    } else {
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                })
                .await;

                // Write what we can without waiting further.
                let mut n = 0usize;
                while n < buf.len() {
                    let ifr = regs.$if_().read();
                    if ifr.txfifo_full().bit_is_set() {
                        break;
                    }
                    regs.$tdr().write(|w| unsafe { w.tdr().bits(buf[n] as u16) });
                    n += 1;
                }

                Ok(n)
            }

            async fn flush(&mut self) -> Result<(), Self::Error> {
                let regs = <$UART as Instance>::regs();

                poll_fn(|cx| {
                    let ifr = regs.$if_().read();
                    if ifr.txbusy().bit_is_clear() {
                        Poll::Ready(())
                    } else {
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                })
                .await;

                Ok(())
            }
        }

        impl<TX> eioa::Write for Tx<$UART, TX> {
            async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                if buf.is_empty() {
                    return Ok(0);
                }

                let regs = <$UART as Instance>::regs();

                poll_fn(|cx| {
                    let ifr = regs.$if_().read();
                    if ifr.txfifo_full().bit_is_clear() {
                        Poll::Ready(())
                    } else {
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                })
                .await;

                let mut n = 0usize;
                while n < buf.len() {
                    let ifr = regs.$if_().read();
                    if ifr.txfifo_full().bit_is_set() {
                        break;
                    }
                    regs.$tdr().write(|w| unsafe { w.tdr().bits(buf[n] as u16) });
                    n += 1;
                }

                Ok(n)
            }

            async fn flush(&mut self) -> Result<(), Self::Error> {
                let regs = <$UART as Instance>::regs();

                poll_fn(|cx| {
                    let ifr = regs.$if_().read();
                    if ifr.txbusy().bit_is_clear() {
                        Poll::Ready(())
                    } else {
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                })
                .await;

                Ok(())
            }
        }
    };
}

// Implement for UART0/1/2 with their PAC register accessor names.
impl_uart_async!(pac::UART0, uart0_tdr, uart0_rdr, uart0_if);
impl_uart_async!(pac::UART1, uart1_tdr, uart1_rdr, uart1_if);
impl_uart_async!(pac::UART2, uart2_tdr, uart2_rdr, uart2_if);

